using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Sensor;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class ADIS16500 : ISPICSPeripheral
    {
        public override void FinishTransmission()
        {
        }

        public override void Reset()
        {
        }

        protected override byte TransmitSelected(byte data)
        {
            return data;
        }
    }
}
