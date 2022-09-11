#![no_std]

use embedded_hal as hal;
use num_enum::{TryFromPrimitive};
use core::convert::TryFrom;

pub struct BMA421<I2C, InterruptPin> {
    i2c: I2C,
    address: I2C_Address,
    interrupt_pin1: Option<InterruptPin>,
    interrupt_pin2: Option<InterruptPin>,
}

#[allow(non_camel_case_types)]
#[repr(u8)]
#[derive(Copy,Clone)]
pub enum I2C_Address {
    /// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bma400-ds000.pdf#page=108
    BMA400 = 0b001_0100,
    /// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bma400-ds000.pdf#page=108
    /// chosen by pin-strapping SDO to VDDIO
    BMA400_ALTERNATIVE = 0b001_0101,
    /// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bma456-ds000.pdf#page=82
    /// also used by BMA421 and BMA425
    BMA456 = 0b001_1000,
    /// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bma456-ds000.pdf#page=82
    /// also used by BMA421 and BMA425
    /// chosen by pin-strapping SDO to VDDIO
    BMA456_ALTERNATIVE = 0b001_1001,
}

/// created per https://github.com/InfiniTimeOrg/InfiniTime/tree/develop/src/drivers
impl<I2C, CommunicationError, InterruptPin> BMA421<I2C, InterruptPin>
    where
        I2C: hal::blocking::i2c::Write<Error = CommunicationError>
            + hal::blocking::i2c::Read<Error = CommunicationError>
            + hal::blocking::i2c::WriteRead<Error = CommunicationError>,
        InterruptPin: hal::digital::v2::InputPin,
{
    /// verifies ability to talk to chip
    pub fn new<'a>(i2c: I2C,
               address: I2C_Address,
               interrupt_pin1: Option<InterruptPin>,
               interrupt_pin2: Option<InterruptPin>,
    ) -> Result<Self, Error<CommunicationError>>
    {
        let mut _self = Self{
            i2c,
            address,
            interrupt_pin1,
            interrupt_pin2,
        };

        // verify chip id
        const BMA421_CHIP_ID:u8 = 0x11;
        let chip_id = _self.read_register(Registers::CHIP_ID)?;
        if chip_id != BMA421_CHIP_ID {
            return Err(Error::UnknownChipId(chip_id))
        }

        Ok(_self)
    }

    pub fn low_power(&mut self, config: Option<LowPowerOverSamplingConfig>) -> Result<(), Error<CommunicationError>> {
        let mode =
            Mode::LowPower as u8
            | match config {
                Some(config) => {
                    const OSR_LP_SHIFT:usize = 6;
                    const FILT1_BW_SHIFT:usize = 7;
                    (1 << OSR_LP_SHIFT) | ((config as u8) << FILT1_BW_SHIFT)
                }
                None => 0
            };

        self.set_mode(mode)
    }

    fn set_mode(&mut self, mode: u8) -> Result<(), Error<CommunicationError>> {
        self.write_register(Registers::ACC_CONF, mode)
    }


    fn drdy(&mut self) -> Result<bool, Error<CommunicationError>> {
        let status = self.read_register(Registers::STATUS)?;
        const DRDY_MASK:u8 = 1 << 7;
        Ok((status & DRDY_MASK) == DRDY_MASK)
    }

    pub fn accelerations(&mut self) -> Result<[i16;3], Error<CommunicationError>> {
        // make sure the data is available
    #[cfg(debug_assertions)]
        if !(self.drdy()?) {
            return Err(Error::DataNotReady)
        }

        // get the data
        let mut buffer:[u8;6] = [0;6];
        self.read_registers(Registers::DATA_START, &mut buffer)?;

        // convert the data
        let mut accelerations:[i16;3] = [0; 3];
        for i in 0..accelerations.len() {
            let index = 2 * i;
            let lsb = buffer[index] as i16;
            let msb = buffer[index + 1] as i16;
            let raw_value = (msb << 8) | lsb;
            accelerations[i] = raw_value - (1<<11);
        }
        Ok(accelerations)
    }

    fn read_register(&mut self, register: Registers ) -> Result<u8, Error<CommunicationError>> {
        let request = &[register as u8];
        let mut response:[u8;1] = [0;1];
        self.i2c.write_read(self.address as u8, request, &mut response).map_err(Error::I2c)?;
        Ok(response[0])
    }

    fn read_registers(&mut self, first_register: Registers, response: &mut [u8] ) -> Result<(), Error<CommunicationError>> {
        let request = &[first_register as u8];
        self.i2c.write_read(self.address as u8, request, response).map_err(Error::I2c)?;
        Ok(())
    }

    fn write_register(&mut self, register: Registers, value: u8 ) -> Result<(), Error<CommunicationError>> {
        let request = &[register as u8, value];
        self.i2c.write(self.address as u8, request).map_err(Error::I2c)?;
        Ok(())
    }

}

#[derive(Debug, Clone, PartialEq)]
pub enum Error<CommunicationError> {
    /// Underlying I2C device error
    I2c(CommunicationError),
    /// unrecognized BMA chip id
    UnknownChipId(u8),
    /// Data not ready (likely chip is asleep)
    DataNotReady,
    /// Device failed to resume from reset
    ResetTimeout
}

#[repr(u8)]
/// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bma400-ds000.pdf#page=40
enum Mode {
    Sleep       = 0x00,
    LowPower    = 0x01,
    Normal      = 0x02,
}

#[repr(u8)]
/// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bma400-ds000.pdf#page=66
pub enum LowPowerOverSamplingConfig {
    HIGH    = 0b00,
    LOW     = 0b01,
}

#[allow(unused)]
#[allow(non_camel_case_types)]
#[repr(u8)]
/// https://github.com/InfiniTimeOrg/InfiniTime/blob/develop/src/drivers/Bma421_C/bma4_defs.h#L105
enum Registers {
    CHIP_ID             = 0x00,
    ERR_REG             = 0x02,
    STATUS              = 0x03,
    DATA_START          = 0x0A,
    DATA_END            = 0x12,
    SENSORTIME_START    = 0x18,
    SENSORTIME_END      = 0x1A,
    INT_STATUS_0        = 0x1C,
    INT_STATUS_1        = 0x1D,
    STEP_COUNTER_START  = 0x1E,
    STEP_COUNTER_END    = 0x21,
    TEMPERATURE         = 0x22,
    FIFO_LENGTH_START   = 0x24,
    FIFO_LENGTH_END     = 0x25,
    FIFO_DATA           = 0x26,
    ACC_CONF            = 0x40,
    ACC_RANGE           = 0x41,
    AUX_CONF            = 0x44,
    FIFO_DOWNS          = 0x45,
    FIFO_WTM_START      = 0x46,
    FIFO_WTM_END        = 0x47,
    FIFO_CONFIG_START   = 0x48,
    FIFO_CONFIG_END     = 0x49,
    AUX_DEV_ID          = 0x4B,
    AUX_IF_CONF         = 0x4C,
    AUX_RD_ADDR         = 0x4D,
    AUX_WR_ADDR         = 0x4E,
    AUX_WR_DATA         = 0x4F,
    INT1_IO_CTRL        = 0x53,
    INT2_IO_CTRL        = 0x54,
    INT_LATCH           = 0x55,
    INT1_MAP            = 0x56,
    INT2_MAP            = 0x57,
    INT_MAP_DATA        = 0x58,
    INIT_CTRL           = 0x59,
    FEATURES_IN         = 0x5E,
    INTERNAL_ERROR      = 0x5F,
    IF_CONF             = 0x6B,
    ACC_SELF_TEST       = 0x6D,
    NV_CONF             = 0x70,
    OFFSET_START        = 0x71,
    OFFSET_END          = 0x73,
    PWR_CONF            = 0x7C,
    PWR_CTRL            = 0x7D,
    CMD                 = 0x7E,
}
