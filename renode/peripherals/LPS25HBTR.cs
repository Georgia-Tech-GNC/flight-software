using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Sensor;
using Antmicro.Renode.Peripherals.I2C;

using System;
using System.Collections.Generic;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class LPS25HBTR : II2CPeripheral
    {
        private readonly LinkedList<byte> _deque = new LinkedList<byte>();

        public LPS25HBTR()
        {
            
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
                this.Log(LogLevel.Warning, $"Requested {count} bytes, but only {_deque.Count} bytes are available. Returning partial data.");
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

            foreach (byte b in data)
            {
                this.Log(LogLevel.Info, $"Received byte 0x{b:X2}");
                _deque.AddLast(b); // Push to the back of the deque
            }
        }

        public void FinishTransmission()
        {
        }

        public void Reset()
        {
        }
    }
}
