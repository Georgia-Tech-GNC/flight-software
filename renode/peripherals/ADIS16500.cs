using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Sensor;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class ADIS16500 : ISPICSPeripheral
    {
        public GPIO IRQ { get; }

        public ADIS16500()
        {
            IRQ = new GPIO();
            IRQ.Set(false);
        }
       
        public override void FinishTransmission()
        {
            IRQ.Blink();
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
