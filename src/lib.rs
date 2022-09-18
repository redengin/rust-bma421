#![no_std]

use embedded_hal as hal;
// use num_enum::{TryFromPrimitive};
// use core::{convert::TryFrom, slice::Windows};
mod firmware;

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
        let chip_id = _self.read_register(Register::CHIP_ID)?;
        if chip_id != BMA421_CHIP_ID {
            return Err(Error::UnknownChipId(chip_id))
        }

        // FIXME is this necessary upon init
        // let firmware = firmware::BLOB;
        // _self.load_firmware(&firmware, delay)?;

        // FIXME is this necessary upon init
        // TODO set interrupt mode

        // FIXME is this necessary upon init
        // TODO feature enable

        // FIXME is this necessary upon init
        // TODO step detector enable

        // FIXME is this necessary upon init
        // TODO accel enable

        Ok(_self)
    }

    /// https://www.mouser.com/datasheet/2/783/BST-BMA423-DS000-1509600.pdf#%5B%7B%22num%22%3A245%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C61%2C725%2C0%5D
    pub fn enable_accelerometer(&mut self) -> Result<(), Error<CommunicationError>> {
        let mut pwr_ctrl = self.read_register(Register::PWR_CTRL)?;
        const ACC_EN_SHIFT:usize = 2;
        pwr_ctrl |= 1 << ACC_EN_SHIFT;
        self.write_register(Register::PWR_CTRL, pwr_ctrl)
    }

    /// blocks until accelerations available
    /// note: enable_accelerometer() must be called beforehand
    pub fn accelerations(&mut self) -> Result<[i16;3], Error<CommunicationError>> {
        // make sure the data is available
        while !(self.drdy()?) { /* wait for data */ }

        // get the data
        let mut buffer:[u8;6] = [0;6];
        self.read_registers(Register::DATA_START, &mut buffer)?;

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

    /// https://www.mouser.com/datasheet/2/783/BST-BMA423-DS000-1509600.pdf#%5B%7B%22num%22%3A245%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C61%2C725%2C0%5D
    pub fn disable_accelerometer(&mut self) -> Result<(), Error<CommunicationError>> {
        let mut pwr_ctrl = self.read_register(Register::PWR_CTRL)?;
        const ACC_EN_SHIFT:usize = 2;
        pwr_ctrl &= !(1 << ACC_EN_SHIFT);
        self.write_register(Register::PWR_CTRL, pwr_ctrl)
    }


    /// load a binary firmware blob into the chip
    pub fn load_firmware<DELAY>(&mut self, firmware: &[u8], delay: &mut DELAY) -> Result<bool, Error<CommunicationError>>
        where
            // TODO doesn't need real-time delay, could use threaded sleep()
            DELAY: embedded_hal::blocking::delay::DelayMs<u16>,
    {
        self.disable_adv_power_save()?;

        // TODO should this be an activity of disable_adv_power_save?
        // wait 450us for time synchronization
        delay.delay_ms(450);

        // disable initialization
        self.write_register(Register::INIT_CTRL, 0)?;

        // write the binary blob
        const BLOCK_LENGTH:usize = 16;
        debug_assert_eq!(0, (firmware.len() % BLOCK_LENGTH), "firmware blob size doesn't match block size");
        let blocks = firmware.len() / BLOCK_LENGTH;
        for block in 0..blocks {
            let index = block * BLOCK_LENGTH;
            let index_msb = ((index / 2) >> 4) as u8;
            let index_lsb = ((index / 2) as u8) & 0x0F;
            self.write_register(Register::MAGIC_5B, index_lsb)?;
            self.write_register(Register::MAGIC_5C, index_msb)?;
            const REQUEST_SIZE:usize = 1 + BLOCK_LENGTH;
            let mut request:[u8;REQUEST_SIZE] = [0;REQUEST_SIZE];
            request[0] = Register::FEATURES_IN as u8;
            request[1..REQUEST_SIZE].copy_from_slice(&firmware[index..index+BLOCK_LENGTH]);
            self.i2c.write(self.address as u8, &request).map_err(Error::I2c)?;
        } 

        // wait for 150us to initialize the ASIC
        delay.delay_ms(150);

        // confirm load
        let status = self.read_register(Register::MAGIC_2A)?;
        const ASIC_INITIALIZED_MASK:u8 = 0b1;
        Ok((status & ASIC_INITIALIZED_MASK) == ASIC_INITIALIZED_MASK)
    }


    /// https://www.mouser.com/datasheet/2/783/BST-BMA423-DS000-1509600.pdf#%5B%7B%22num%22%3A242%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C61%2C442%2C0%5D
    pub fn disable_adv_power_save(&mut self) -> Result<(), Error<CommunicationError>> {
        self.write_register(Register::PWR_CONF, 0)
    }

    /// https://www.mouser.com/datasheet/2/783/BST-BMA423-DS000-1509600.pdf#%5B%7B%22num%22%3A242%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C61%2C442%2C0%5D
    pub fn enable_adv_power_save(&mut self, fifo_self_wakeup: bool) -> Result<(), Error<CommunicationError>> {
        const FIFO_SELF_WAKEUP_SHIFT:usize = 1;
        let value:u8 = 0b1  // enable adv_power_save
            // choose fifo_self_wakeup config
            | (if fifo_self_wakeup { 1 } else {0}) << FIFO_SELF_WAKEUP_SHIFT;
        self.write_register(Register::PWR_CONF, value)
    }

    /// put the chip into sleep
    pub fn sleep(&mut self) -> Result<(), Error<CommunicationError>> {
        self.write_register(Register::ACC_CONF, Mode::Normal as u8)
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

        self.write_register(Register::ACC_CONF, mode)
    }


    /// check if there is data
    fn drdy(&mut self) -> Result<bool, Error<CommunicationError>> {
        let status = self.read_register(Register::STATUS)?;
        const DRDY_MASK:u8 = 1 << 7;
        Ok((status & DRDY_MASK) == DRDY_MASK)
    }

// helpers for I2C
//--------------------------------------------------------------------------------
    fn read_register(&mut self, register: Register ) -> Result<u8, Error<CommunicationError>> {
        let mut response:[u8;1] = [0;1];
        self.read_registers(register, &mut response)?;
        Ok(response[0])
    }

    fn read_registers(&mut self, first_register: Register, response: &mut [u8] ) -> Result<(), Error<CommunicationError>> {
        let request = &[first_register as u8];
        self.i2c.write_read(self.address as u8, request, response).map_err(Error::I2c)?;
        Ok(())
    }

    fn write_register(&mut self, register: Register, value: u8 ) -> Result<(), Error<CommunicationError>> {
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
enum Register {
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
    MAGIC_2A            = 0x2A,
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
    /// used to program firmware
    MAGIC_5B            = 0x5B,
    /// used to program firmware
    MAGIC_5C            = 0x5C,
    /// magic burst writable address
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
