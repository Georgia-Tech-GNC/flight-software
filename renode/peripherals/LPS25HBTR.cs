using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Sensor;
using Antmicro.Renode.Peripherals.I2C;

using System;
using System.Text;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class LPS25HBTR : II2CPeripheral
    {
        private class Register 
        {
            public static Register WhoAmI = new Register(0x0F, 1, new byte[] { 0xBD });
            public static Register Temperature = new Register(0x2B, 2, new byte[2]);
            public static Register Pressure = new Register(0x28, 3, new byte[3]);
            
            public readonly byte Address;
            
            private byte[] _Value;
            public byte[] Value 
            { 
                get => _Value; 
                set 
                {
                    if (_Value is not null && _Value.Length != Size)
                    {
                        Logger.Log(
                            LogLevel.Error, 
                            $"i2c1.lps25hbtr: Register {Address:X} was attempted to be written to with incorrect length. " 
                                + $"Expected value.Length = {Size}, got value.Length = {value.Length}."
                        );
                        
                        return;
                    }
                    
                    _Value = value;
                }
            }
            
            public readonly int Size;
            
            public Register(byte address, int size, byte[] value) 
            {
                Address = address;
                Size = size;
                Value = value;
            }
        }
        
        private const byte Address = 89;
        
        private const float TemperatureCelsiusToNative = 480.0f;
        private const float TemperatureNativeToCelsius =  1.0f / TemperatureCelsiusToNative;
        
        private const float PressureHectopascalsToNative = 4096.0f;
        private const float PressureNativeToHectopascals = 1.0f / PressureHectopascalsToNative;
        
        public float Temperature
        { 
            get => BytesToTemperature(Register.Temperature.Value);
            set => Register.Temperature.Value = TemperatureToBytes(value);
        }
        
        public float Pressure
        {
            get => BytesToPressure(Register.Pressure.Value);
            set => Register.Pressure.Value = PressureToBytes(value);
        }
        
        private readonly LinkedList<byte> _deque = new LinkedList<byte>();
        
        public LPS25HBTR()
        {
            Temperature = 50.0f;
            Pressure = 1013.25f;
        }

        public byte[] Read(int count = 1)
        {
            // 1. Guard against invalid count requests
            if (count < 0)
            {
                this.Log(LogLevel.Error, "Read count cannot be negative.");
                return Array.Empty<byte>();
            }

            // 2. Check if the deque has enough items, warning the user if it doesn't
            if (count > _deque.Count)
            {
                this.Log(
                    LogLevel.Warning, 
                    $"Requested {count} bytes, but only {_deque.Count} bytes are available."
                        + "Returning partial data."
                );
                // Cap the count to what is actually available to avoid crashes
                count = _deque.Count;
            }

            // 3. Extract the items from the front of the deque
            byte[] result = new byte[count];
            for (int i = 0; i < count; i++)
            {
                result[i] = _deque.First.Value;
                _deque.RemoveFirst(); // Pop from the front
            }
            
            return result;
        }

        public void Write(byte[] data)
        {
            if (data == null || data.Length == 0) return;
            
            this.Log(LogLevel.Info, $"Written Bytes: {FormatArray(data)}");
            
            byte addr = (byte)(data[0] & 0x7F);
            
            if (data[1] != (Address << 1 | 1))
            {
                this.Log(
                    LogLevel.Error,
                    $"Invalid read. Expected data[1] = SAD+R = 0x{((Address << 1) | 1):X2}"
                        + $", got 0x{data[1]:X2}."
                );
                
                return;
            }
            
            if (addr == Register.WhoAmI.Address)
            {
                this.Log(
                    LogLevel.Info,
                    $"WhoAmI requested. Register.WhoAmI.Value: {FormatArray(Register.WhoAmI.Value)}"
                );
                
                AddBytes(Register.WhoAmI.Value);
                
                return;
            }
                
            if (addr == Register.Temperature.Address)
            {
                this.Log(
                    LogLevel.Info, 
                    $"Temperature requested. Register.Temperature.Value: {FormatArray(Register.Temperature.Value)} "
                        + $"= {Temperature} celsius"
                );
                
                AddBytes(Register.Temperature.Value);
                
                return;
            }
            
            if (addr == Register.Pressure.Address)
            {
                this.Log(
                    LogLevel.Info,
                    $"Pressure requested. Register.Pressure.Value: {FormatArray(Register.Pressure.Value)} "
                        + $"= {Pressure} hPa"
                );
                
                AddBytes(Register.Pressure.Value);
                
                return;
            }
            
            this.Log(LogLevel.Warning, $"Unrecognized address ({addr:X})");
        }

        public void FinishTransmission() {}

        public void Reset() {}
        
        private byte[] TemperatureToBytes(float temperature)
        {
            UInt16 native = (UInt16)(temperature * TemperatureCelsiusToNative);
            UInt16 nativeTwosComplement = (UInt16)unchecked(~native + 1);
            
            this.Log(LogLevel.Info, $"Native={native}, NativeTwosComplement={nativeTwosComplement}");
            
            return BitConverter.GetBytes(nativeTwosComplement);
        }
        
        private float BytesToTemperature(byte[] bytes)
        {
            if (bytes.Length != 2)
            {
                this.Log(
                    LogLevel.Error, 
                    $"BytesToTemperature: byte array must be of length 2. bytes.Length = {bytes.Length}"
                );
                
                return 0.0f;
            }
            
            UInt16 native = (UInt16)unchecked(~BitConverter.ToUInt16(bytes) + 1);
            return native * TemperatureNativeToCelsius;
        }
        
        private byte[] PressureToBytes(float pressure) 
        {
            UInt32 native = (UInt32)(pressure * PressureHectopascalsToNative);
            UInt32 nativeTwosComplement = unchecked(~native + 1);
            
            byte[] bytes = BitConverter.GetBytes(nativeTwosComplement)[0..3];
            
            return bytes;
        }
        
        private float BytesToPressure(byte[] bytes)
        {
            if (bytes.Length != 3) 
            {
                this.Log(
                    LogLevel.Error,
                    $"BytesToPressure: byte array must be of length 3. bytes.Length = {bytes.Length}"
                );
                
                return 0.0f;
            }
            
            byte[] fourBytes = new byte[] { bytes[0], bytes[1], bytes[2], 0x00 };
                
            UInt32 native = unchecked(~BitConverter.ToUInt32(fourBytes) + 1) & 0x00FFFFFF;
            
            return native * PressureNativeToHectopascals;
        }
        
        private void AddBytes(byte[] bytes) { foreach (byte b in bytes) _deque.AddLast(b); }
        
        private static String FormatArray(byte[] bytes) {
            StringBuilder sb = new StringBuilder();
            
            sb.Append("[");
            for (int i = 0; i < bytes.Length; i++) 
            {
                if (i < bytes.Length - 1) 
                    sb.Append($"0x{bytes[i]:X2}, ");
                else 
                    sb.Append($"0x{bytes[i]:X2}]");
            }
            
            return sb.ToString();
        }
    }
}
