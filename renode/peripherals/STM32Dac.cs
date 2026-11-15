
//
// Copyright (c) 2026
//
// This file is licensed under the MIT License.
//

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Peripherals.Analog;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Logging;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{

    
    public class STM32Dac :
        NullRegistrationPointPeripheralContainer<IVoltageSink>,
        IDoubleWordPeripheral,
        IWordPeripheral,
        IKnownSize,
        IGPIOReceiver
    {
        
        public GPIO DMAReceive { get; }
        public STM32Dac(IMachine machine) : base(machine)
        {
            registers = new DoubleWordRegisterCollection(this);
            DMAReceive = new GPIO();
            DefineRegisters();
            Reset();
        }

        public override void Reset()
        {
            registers.Reset();
            UpdateOutput();
        }

        public void OnGPIO(int number, bool value)
        {
            if (!lastValue && value)
            {
                DMAReceive.Blink();
            }

            lastValue = value;   
        }

        public ushort ReadWord(long offset)
        {
            return (ushort)registers.Read(offset);
        }

        public void WriteWord(long offset, ushort value)
        {
            registers.Write(offset, value);
        }

        public uint ReadDoubleWord(long offset)
        {
            return registers.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            registers.Write(offset, value);
        }

        public long Size => 0x400;

        private void DefineRegisters()
        {
            registers.DefineRegister(0x08)
                .WithValueField(
                    0,
                    12,
                    out dacValue,
                    name: "DHR12R"
                )
                .WithWriteCallback((_, __) =>
                {
                    UpdateOutput();
                });
            registers.DefineRegister(0x00)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => 0);

            registers.DefineRegister(0x34)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ => 0);
        }

        private void UpdateOutput()
{
    var voltage = 3.3 * dacValue.Value / 4096.0;
    RegisteredPeripheral?.SetVoltage(voltage);
}

        private readonly DoubleWordRegisterCollection registers;

        private IValueRegisterField dacValue;

        private bool lastValue = false;
    }
}
