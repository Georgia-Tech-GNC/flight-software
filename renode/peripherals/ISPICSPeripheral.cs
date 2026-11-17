using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Sensor;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.SPI
{
    public abstract class ISPICSPeripheral : ISPIPeripheral, IGPIOReceiver
    {
        private bool chipSelectActive = false;

        public void OnGPIO(int number, bool value)
        {
            chipSelectActive = !value;
        }

        public byte Transmit(byte data)
        {
            if (!chipSelectActive)
            {
                return 0xFF; 
            } else
            {
                return TransmitSelected(data);
            }
        }

        protected abstract byte TransmitSelected(byte data);
        public abstract void FinishTransmission();
        public abstract void Reset();
    }
}
