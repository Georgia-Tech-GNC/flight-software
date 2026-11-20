using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Sensor;
using Antmicro.Renode.Peripherals.I2C;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class LSM6DSV : II2CPeripheral
    {
        public void Write(byte[] data)
        {
        }

        public byte[] Read(int count = 1)
        {
            return new byte[count];
        }

        public void FinishTransmission()
        {
        }

        public void Reset()
        {
        }
    }
}
