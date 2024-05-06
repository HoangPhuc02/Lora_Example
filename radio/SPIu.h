
class SPIu : public SPI
{
public:
    SPIu(PinName mosi, PinName miso, PinName sclk, PinName ssel=NC);
    virtual void lock(void);
    virtual void unlock(void);
};
