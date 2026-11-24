using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    // Simulation of the ST LSM6DSV 6-axis IMU on I2C: output registers, self-test, timestamp and
    // raw accel/gyro batching into the FIFO. Embedded functions (sensor fusion) are not modeled.
    //
    // Simulated inputs, settable from the monitor or a robot test:
    //   AccelerationX/Y/Z [g], AngularRateX/Y/Z [dps], GyroBiasX/Y/Z [dps], Temperature [degC],
    //   SelfTestFault (makes the built-in self-test produce no output change)
    //
    // The gyro output reports AngularRate + GyroBias. FIFO samples are produced lazily from virtual
    // time, so the FIFO holds whatever the real chip would have batched since the last access.
    public class LSM6DSV : II2CPeripheral
    {
        public LSM6DSV(IMachine machine)
        {
            this.machine = machine;
            Reset();
        }

        public double AccelerationX { get { return Get(accel, 0); } set { Set(accel, 0, value); } }
        public double AccelerationY { get { return Get(accel, 1); } set { Set(accel, 1, value); } }
        public double AccelerationZ { get { return Get(accel, 2); } set { Set(accel, 2, value); } }

        public double AngularRateX { get { return Get(rate, 0); } set { Set(rate, 0, value); } }
        public double AngularRateY { get { return Get(rate, 1); } set { Set(rate, 1, value); } }
        public double AngularRateZ { get { return Get(rate, 2); } set { Set(rate, 2, value); } }

        public double GyroBiasX { get { return Get(bias, 0); } set { Set(bias, 0, value); } }
        public double GyroBiasY { get { return Get(bias, 1); } set { Set(bias, 1, value); } }
        public double GyroBiasZ { get { return Get(bias, 2); } set { Set(bias, 2, value); } }

        public double Temperature { get; set; }

        public bool SelfTestFault { get; set; }

        public int FifoCount
        {
            get
            {
                lock(sync)
                {
                    Advance();
                    return fifo.Count;
                }
            }
        }

        public void Reset()
        {
            lock(sync)
            {
                Array.Clear(mainRegisters, 0, mainRegisters.Length);
                mainRegisters[PinCtrl] = 0x23;
                mainRegisters[IfCfg] = 0x00;
                ResetControlRegisters();

                // At rest, Z up
                for(int axis = 0; axis < 3; axis++)
                {
                    accel[axis] = axis == 2 ? 1 : 0;
                    rate[axis] = 0;
                    bias[axis] = 0;
                }
                Temperature = 25;
                SelfTestFault = false;

                lastAdvanceNs = NowNs();
                timestampStartNs = lastAdvanceNs;
                inAddressPhase = true;
            }
        }

        public void FinishTransmission()
        {
            // The next write after a STOP starts with a register address
            lock(sync)
            {
                inAddressPhase = true;
            }
        }

        public void Write(byte[] data)
        {
            lock(sync)
            {
                foreach(byte value in data)
                {
                    if(inAddressPhase)
                    {
                        Advance();
                        address = (byte)(value & 0x7F);
                        inAddressPhase = false;
                        continue;
                    }
                    WriteRegister(address, value);
                    NextAddress();
                }
            }
        }

        public byte[] Read(int count = 1)
        {
            lock(sync)
            {
                var result = new byte[Math.Max(count, 0)];
                for(int i = 0; i < result.Length; i++)
                {
                    result[i] = ReadRegister(address);
                    NextAddress();
                }
                return result;
            }
        }

        private void NextAddress()
        {
            if((mainRegisters[Ctrl3] & Ctrl3IfInc) != 0)
            {
                // Burst reads of the FIFO wrap from FIFO_DATA_OUT_Z_H back to the next tag
                address = (address == FifoDataOutLast) ? FifoDataOutTag : (byte)((address + 1) & 0x7F);
            }
        }

        // ------------------------------------------------------------------------------------------
        // Register access
        // ------------------------------------------------------------------------------------------

        private bool OtherBank { get { return (mainRegisters[FuncCfgAccess] & (FuncCfgEmbFuncRegAccess | FuncCfgShubRegAccess)) != 0; } }

        private byte ReadRegister(byte reg)
        {
            // FUNC_CFG_ACCESS is visible from every bank so software can switch back
            if(reg == FuncCfgAccess)
            {
                return mainRegisters[reg];
            }
            if(OtherBank)
            {
                this.Log(LogLevel.Warning, "Embedded function and sensor hub registers are not modeled (read 0x{0:X2})", reg);
                return 0;
            }

            switch(reg)
            {
                case WhoAmI:
                    return WhoAmIValue;
                case FifoStatus1:
                    Advance();
                    return (byte)(fifo.Count & 0xFF);
                case FifoStatus2:
                    return ReadFifoStatus2();
                case StatusReg:
                    return ReadStatus();
                case FifoDataOutTag:
                    currentWord = fifo.Count > 0 ? fifo.Dequeue() : new byte[FifoWordBytes];
                    return currentWord[0];
                case TimestampFirst:
                    LatchTimestamp();
                    return timestampLatch[0];
            }

            if(reg >= OutTempL && reg <= OutzHA)
            {
                return ReadOutput(reg);
            }
            if(reg > TimestampFirst && reg <= TimestampLast)
            {
                return timestampLatch[reg - TimestampFirst];
            }
            if(reg > FifoDataOutTag && reg <= FifoDataOutLast)
            {
                return currentWord[reg - FifoDataOutTag];
            }
            return mainRegisters[reg];
        }

        private void WriteRegister(byte reg, byte value)
        {
            if(reg == FuncCfgAccess)
            {
                if((value & FuncCfgSwPor) != 0)
                {
                    this.Log(LogLevel.Info, "SW_POR: global reset");
                    Reset();
                    return;
                }
                mainRegisters[reg] = value;
                return;
            }
            if(OtherBank)
            {
                this.Log(LogLevel.Warning, "Embedded function and sensor hub registers are not modeled (write 0x{0:X2} = 0x{1:X2})", reg, value);
                return;
            }

            if(reg == WhoAmI || (reg >= CtrlStatus && reg <= InternalFreqFine) || (reg >= FifoDataOutTag && reg <= FifoDataOutLast))
            {
                this.Log(LogLevel.Warning, "Write to read-only register 0x{0:X2} ignored", reg);
                return;
            }

            switch(reg)
            {
                case Ctrl3:
                    if((value & Ctrl3SwReset) != 0)
                    {
                        Advance();
                        ResetControlRegisters();
                        return;
                    }
                    // BOOT clears itself once the (instant) reboot is done
                    mainRegisters[reg] = (byte)(value & ~Ctrl3Boot);
                    return;
                case Ctrl1:
                case Ctrl2:
                case FifoCtrl3:
                case FifoCtrl4:
                    Advance();
                    mainRegisters[reg] = value;
                    if(reg == FifoCtrl4 && (value & FifoModeMask) == FifoModeBypass)
                    {
                        fifo.Clear();
                        overrunLatched = false;
                    }
                    return;
                case FunctionsEnable:
                    if((value & TimestampEn) != 0 && (mainRegisters[reg] & TimestampEn) == 0)
                    {
                        timestampStartNs = NowNs();
                    }
                    mainRegisters[reg] = value;
                    return;
                default:
                    mainRegisters[reg] = value;
                    return;
            }
        }

        private void ResetControlRegisters()
        {
            // PIN_CTRL and IF_CFG survive a software reset; everything else returns to defaults
            byte pinCtrl = mainRegisters[PinCtrl];
            byte ifCfg = mainRegisters[IfCfg];
            Array.Clear(mainRegisters, 0, mainRegisters.Length);
            mainRegisters[PinCtrl] = pinCtrl;
            mainRegisters[IfCfg] = ifCfg;
            mainRegisters[Ctrl3] = Ctrl3Bdu | Ctrl3IfInc;
            mainRegisters[Den] = 0x0E;
            mainRegisters[InactivityDur] = 0x04;
            fifo.Clear();
            overrunLatched = false;
            currentWord = new byte[FifoWordBytes];
        }

        // ------------------------------------------------------------------------------------------
        // Output registers
        // ------------------------------------------------------------------------------------------

        private int AccelOdrCode { get { return mainRegisters[Ctrl1] & OdrMask; } }
        private int GyroOdrCode { get { return mainRegisters[Ctrl2] & OdrMask; } }

        private double AccelSensitivityMg { get { return 0.061 * (1 << (mainRegisters[Ctrl8] & 0x03)); } }

        private double GyroSensitivityMdps
        {
            get
            {
                int fs = mainRegisters[Ctrl6] & 0x0F;
                return fs == GyroFs4000 ? 140.0 : 4.375 * (1 << Math.Min(fs, 4));
            }
        }

        private byte ReadStatus()
        {
            byte status = 0;
            if(AccelOdrCode != 0)
            {
                status |= StatusXlda | StatusTda;
            }
            if(GyroOdrCode != 0)
            {
                status |= StatusGda | StatusTda;
            }
            return status;
        }

        private byte ReadOutput(byte reg)
        {
            int index = (reg - OutTempL) / 2;
            short raw;
            if(index == 0)
            {
                raw = ToInt16((Temperature - 25.0) * 256.0);
            }
            else if(index <= 3)
            {
                raw = GyroOdrCode == 0 ? (short)0 : GyroCount(index - 1);
            }
            else
            {
                raw = AccelOdrCode == 0 ? (short)0 : AccelCount(index - 4);
            }
            return (byte)(((reg - OutTempL) % 2 == 0) ? (raw & 0xFF) : ((raw >> 8) & 0xFF));
        }

        private short GyroCount(int axis)
        {
            return ToInt16((rate[axis] + bias[axis] + GyroSelfTestDps()) * 1000.0 / GyroSensitivityMdps);
        }

        private short AccelCount(int axis)
        {
            return ToInt16((accel[axis] + AccelSelfTestG()) * 1000.0 / AccelSensitivityMg);
        }

        // Self-test forces sized to land inside the datasheet pass windows
        private double AccelSelfTestG()
        {
            return SelfTestSign(mainRegisters[Ctrl10] & 0x03) * (SelfTestFault ? 0.0 : 0.5);
        }

        private double GyroSelfTestDps()
        {
            double force = GyroSensitivityMdps >= 70.0 ? 300.0 : 50.0;
            return SelfTestSign((mainRegisters[Ctrl10] >> 2) & 0x03) * (SelfTestFault ? 0.0 : force);
        }

        private static int SelfTestSign(int bits)
        {
            return bits == 1 ? 1 : (bits == 2 ? -1 : 0);
        }

        private void LatchTimestamp()
        {
            uint ticks = 0;
            if((mainRegisters[FunctionsEnable] & TimestampEn) != 0)
            {
                ticks = (uint)((ulong)((NowNs() - timestampStartNs) / TimestampTickNs) & 0xFFFFFFFF);
            }
            timestampLatch = BitConverter.GetBytes(ticks);
            if(!BitConverter.IsLittleEndian)
            {
                Array.Reverse(timestampLatch);
            }
        }

        // ------------------------------------------------------------------------------------------
        // FIFO
        // ------------------------------------------------------------------------------------------

        private byte ReadFifoStatus2()
        {
            int count = fifo.Count;
            int watermark = mainRegisters[FifoCtrl1];
            byte status = (byte)((count >> 8) & FifoStatus2DiffFifo8);
            if(watermark > 0 && count >= watermark)
            {
                status |= FifoStatus2WtmIa;
            }
            if(count >= FifoCapacity)
            {
                status |= FifoStatus2OvrIa;
            }
            if(count >= FifoCapacity - 1)
            {
                status |= FifoStatus2FullIa;
            }
            if(overrunLatched)
            {
                status |= FifoStatus2OvrLatched;
                overrunLatched = false; // cleared on read
            }
            return status;
        }

        private void PushFifo(byte tag, short x, short y, short z)
        {
            int mode = mainRegisters[FifoCtrl4] & FifoModeMask;
            if(mode == FifoModeBypass)
            {
                return;
            }
            if(fifo.Count >= FifoCapacity)
            {
                overrunLatched = true;
                if(mode == FifoModeFifo)
                {
                    return; // FIFO mode stops collecting when full
                }
                fifo.Dequeue(); // continuous mode overwrites the oldest entry
            }

            byte[] word = new byte[FifoWordBytes];
            word[0] = (byte)(tag << 3);
            word[1] = (byte)(x & 0xFF);
            word[2] = (byte)((x >> 8) & 0xFF);
            word[3] = (byte)(y & 0xFF);
            word[4] = (byte)((y >> 8) & 0xFF);
            word[5] = (byte)(z & 0xFF);
            word[6] = (byte)((z >> 8) & 0xFF);
            fifo.Enqueue(word);
        }

        private static ulong BatchPeriodNs(int bdrCode, int odrCode)
        {
            int code = Math.Min(bdrCode, odrCode);
            double hz = code <= 1 ? 1.875 : 7.5 * (1 << (code - 2));
            return (ulong)(1e9 / hz);
        }

        private void BatchSensor(int bdrCode, int odrCode, ref int activeCode, ref ulong nextNs, ulong now, byte tag, Func<int, short> count)
        {
            if(bdrCode == 0 || bdrCode > 0x0C)
            {
                activeCode = 0;
                return;
            }
            ulong period = BatchPeriodNs(bdrCode, odrCode);
            if(activeCode != bdrCode)
            {
                activeCode = bdrCode;
                nextNs = lastAdvanceNs + period;
            }
            int steps = 0;
            while(nextNs <= now)
            {
                PushFifo(tag, count(0), count(1), count(2));
                nextNs += period;
                if(++steps > 4 * FifoCapacity)
                {
                    // Nobody read the FIFO for a long time; it is already full, so skip ahead
                    nextNs = now + period;
                    break;
                }
            }
        }

        // Brings the FIFO up to the current virtual time, batching every sample the chip would
        // have produced since the last call
        private void Advance()
        {
            ulong now = NowNs();
            if(now <= lastAdvanceNs)
            {
                return;
            }

            int gyroCode = GyroOdrCode == 0 ? 0 : (mainRegisters[FifoCtrl3] >> 4) & 0x0F;
            int accelCode = AccelOdrCode == 0 ? 0 : mainRegisters[FifoCtrl3] & 0x0F;
            BatchSensor(gyroCode, GyroOdrCode, ref gyroBatchCode, ref nextGyroBatchNs, now, TagGyroNc, GyroCount);
            BatchSensor(accelCode, AccelOdrCode, ref accelBatchCode, ref nextAccelBatchNs, now, TagAccelNc, AccelCount);
            lastAdvanceNs = now;
        }

        // ------------------------------------------------------------------------------------------
        // Helpers
        // ------------------------------------------------------------------------------------------

        private ulong NowNs()
        {
            return (ulong)machine.LocalTimeSource.ElapsedVirtualTime.TotalNanoseconds;
        }

        private double Get(double[] values, int axis)
        {
            lock(sync)
            {
                return values[axis];
            }
        }

        private void Set(double[] values, int axis, double value)
        {
            lock(sync)
            {
                // Samples batched before the change keep the old value
                Advance();
                values[axis] = value;
            }
        }

        private static short ToInt16(double value)
        {
            double rounded = Math.Round(value);
            if(rounded > short.MaxValue)
            {
                return short.MaxValue;
            }
            if(rounded < short.MinValue)
            {
                return short.MinValue;
            }
            return (short)rounded;
        }

        private readonly IMachine machine;
        private readonly object sync = new object();
        private readonly byte[] mainRegisters = new byte[128];
        private readonly Queue<byte[]> fifo = new Queue<byte[]>();

        private readonly double[] accel = new double[3];
        private readonly double[] rate = new double[3];
        private readonly double[] bias = new double[3];

        private bool inAddressPhase = true;
        private byte address;
        private byte[] currentWord = new byte[FifoWordBytes];
        private byte[] timestampLatch = new byte[4];
        private bool overrunLatched;
        private int gyroBatchCode;
        private int accelBatchCode;
        private ulong nextGyroBatchNs;
        private ulong nextAccelBatchNs;
        private ulong lastAdvanceNs;
        private ulong timestampStartNs;

        private const double TimestampTickNs = 1e9 / 46080.0;
        private const int FifoWordBytes = 7;
        private const int FifoCapacity = 1536 / FifoWordBytes; // 1.5 KB uncompressed
        private const byte WhoAmIValue = 0x70;

        // Registers
        private const byte FuncCfgAccess = 0x01;
        private const byte PinCtrl = 0x02;
        private const byte IfCfg = 0x03;
        private const byte FifoCtrl1 = 0x07;
        private const byte FifoCtrl3 = 0x09;
        private const byte FifoCtrl4 = 0x0A;
        private const byte WhoAmI = 0x0F;
        private const byte Ctrl1 = 0x10;
        private const byte Ctrl2 = 0x11;
        private const byte Ctrl3 = 0x12;
        private const byte Ctrl6 = 0x15;
        private const byte Ctrl8 = 0x17;
        private const byte Ctrl10 = 0x19;
        private const byte CtrlStatus = 0x1A;
        private const byte FifoStatus1 = 0x1B;
        private const byte FifoStatus2 = 0x1C;
        private const byte StatusReg = 0x1E;
        private const byte OutTempL = 0x20;
        private const byte OutzHA = 0x2D;
        private const byte TimestampFirst = 0x40;
        private const byte TimestampLast = 0x43;
        private const byte InternalFreqFine = 0x4F;
        private const byte FunctionsEnable = 0x50;
        private const byte Den = 0x51;
        private const byte InactivityDur = 0x54;
        private const byte FifoDataOutTag = 0x78;
        private const byte FifoDataOutLast = 0x7E;

        // Bits
        private const byte FuncCfgEmbFuncRegAccess = 0x80;
        private const byte FuncCfgShubRegAccess = 0x40;
        private const byte FuncCfgSwPor = 0x04;
        private const byte Ctrl3Boot = 0x80;
        private const byte Ctrl3Bdu = 0x40;
        private const byte Ctrl3IfInc = 0x04;
        private const byte Ctrl3SwReset = 0x01;
        private const byte OdrMask = 0x0F;
        private const int GyroFs4000 = 0x0C;
        private const byte StatusXlda = 0x01;
        private const byte StatusGda = 0x02;
        private const byte StatusTda = 0x04;
        private const byte FifoModeMask = 0x07;
        private const byte FifoModeBypass = 0x00;
        private const byte FifoModeFifo = 0x01;
        private const byte FifoStatus2WtmIa = 0x80;
        private const byte FifoStatus2OvrIa = 0x40;
        private const byte FifoStatus2FullIa = 0x20;
        private const byte FifoStatus2OvrLatched = 0x08;
        private const byte FifoStatus2DiffFifo8 = 0x01;
        private const byte TimestampEn = 0x40;

        // FIFO tags (datasheet table 210)
        private const byte TagGyroNc = 0x01;
        private const byte TagAccelNc = 0x02;
    }
}
