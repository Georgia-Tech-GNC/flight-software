using Antmicro.Renode.Peripherals;

namespace Antmicro.Renode.Peripherals.Analog
{
    public interface IVoltageSink : IPeripheral
    {
        void SetVoltage(double voltage);
    }
}
