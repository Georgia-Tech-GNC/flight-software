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
        public byte[] Read(int count = 1)
        {
            byte[] result = new byte[count];
            for (int i = 0; i < count; i ++)
            {
                result[i] = (byte) i;
            }

            return result;
        }

        public void Write(byte[] data)
        {
            
        }

        public void FinishTransmission()
        {
        }

        public void Reset()
        {
        }
    }
}
