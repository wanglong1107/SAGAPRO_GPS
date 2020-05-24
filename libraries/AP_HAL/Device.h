/*
 * Copyright (C) 2015-2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <inttypes.h>
#include <stdint.h>
#include "AP_HAL_Namespace.h"
//#include "utility/functor.h"

/*
 * This is an interface abstracting I2C and SPI devices
 */
class AP_HAL::Device
{
public:
    enum BusType
	{
        BUS_TYPE_UNKNOWN = 0,
        BUS_TYPE_I2C     = 1,
        BUS_TYPE_SPI     = 2,
        BUS_TYPE_UAVCAN  = 3
    };

    enum Speed 
	{
        SPEED_HIGH,
        SPEED_LOW,
    };
	
//    FUNCTOR_TYPEDEF(PeriodicCb, void);
    typedef void* PeriodicHandle;
	
    Device(enum BusType type)
    {
        _bus_id.devid_s.bus_type = type;
    }

    // return bus type
    enum BusType bus_type(void) const
	{
        return _bus_id.devid_s.bus_type;
    }

    // return bus number
    uint8_t bus_num(void) const
	{
        return _bus_id.devid_s.bus;
    }

    // return 24 bit bus identifier
    uint32_t get_bus_id(void) const
	{
        return _bus_id.devid;
    }

    // return address on bus
    uint8_t get_bus_address(void) const
	{
        return _bus_id.devid_s.address;
    }

    // set device type within a device class (eg. AP_COMPASS_TYPE_LSM303D)
    void set_device_type(uint8_t devtype)
	{
        _bus_id.devid_s.devtype = devtype;
    }


    virtual ~Device() 
	{
        if (_checked.regs != nullptr) {
            delete[] _checked.regs;
        }
    }

	virtual bool initialize()  = 0;
	
    virtual void set_address(uint8_t address) {};
    
    virtual bool set_speed(Speed speed)  = 0;
	
	
	virtual bool writeRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len) = 0;
	virtual bool readRegisters(uint8_t _reg, uint8_t *_pData, uint32_t len) = 0;

	uint8_t read_register(uint8_t _reg)
	{
		uint8_t readData = 0x00;
		uint8_t reg = _reg | _read_flag;
		
		readRegisters( reg, &readData, 1);
		
		return readData;
	}
    bool read_registers(uint8_t first_reg, uint8_t *recv, uint32_t recv_len)
    {
		uint8_t reg = first_reg | _read_flag;
		
		return readRegisters( reg, recv, recv_len);
    }

    bool write_register(uint8_t reg, uint8_t val, bool checked=false)
    {
        uint8_t value = val;
        if (checked) 
		{
            set_checked_register(reg, val);
        }
        return writeRegisters( reg, &value, 1);
    }
	
	bool write_registers(uint8_t reg, uint8_t *_pData, uint32_t _len)
    {
        return writeRegisters( reg, _pData, _len);
    }

    /**
     * set a value for a checked register
     */
    void set_checked_register(uint8_t reg, uint8_t val);

    /**
     * setup for register value checking. Frequency is how often to check registers. If set to 10 then
     * every 10th call to check_next_register will check a register
     */
    bool setup_checked_registers(uint8_t num_regs, uint8_t frequency=10);

    /**
     * check next register value for correctness. Return false if value is incorrect
     * or register checking has not been setup
     */
    bool check_next_register(void);
	
	/**
	  * check next register value for correctness. Return false if value is incorrect
	  * or register checking has not been setup
	  */	
	virtual AP_HAL::Semaphore *get_semaphore() = 0;
	

    /**
     * Wrapper function over #transfer() to read a sequence of bytes from
     * device. No value is written, differently from the #read_registers()
     * method and hence doesn't include the read flag set by #set_read_flag()
     */
//    bool read(uint8_t *recv, uint32_t recv_len)
//    {
//        return transfer(nullptr, 0, recv, recv_len);
//    }


    
    /*
     * support for direct control of SPI chip select. Needed for
     * devices with unusual SPI transfer patterns that include
     * specific delays
     */
    virtual bool set_chip_select(bool set) { return false; }

    /**
     * Some devices connected on the I2C or SPI bus require a bit to be set on
     * the register address in order to perform a read operation. This sets a
     * flag to be used by #read_registers(). The flag's default value is zero.
     */
    void set_read_flag(uint8_t flag)
    {
        _read_flag = flag;
    }


    /**
     * make a bus id given bus type, bus number, bus address and
     * device type This is for use by devices that do not use one of
     * the standard HAL Device types, such as UAVCAN devices
     */
    static uint32_t make_bus_id(enum BusType bus_type, uint8_t bus, uint8_t address, uint8_t devtype)
	{
        union DeviceId d;
        d.devid_s.bus_type = bus_type;
        d.devid_s.bus = bus;
        d.devid_s.address = address;
        d.devid_s.devtype = devtype;
        return d.devid;
    }

    /**
     * return a new bus ID for the same bus connection but a new device type.
     * This is used for auxillary bus connections
     */
    static uint32_t change_bus_id(uint32_t old_id, uint8_t devtype)
	{
        union DeviceId d;
        d.devid = old_id;
        d.devid_s.devtype = devtype;
        return d.devid;
    }

    /**
     * return bus ID with a new devtype
     */
    uint32_t get_bus_id_devtype(uint8_t devtype)
	{
        return change_bus_id(get_bus_id(), devtype);
    }

    /* set number of retries on transfers */
    virtual void set_retries(uint8_t retries) {};

protected:
    uint8_t _read_flag = 0;

    /*
      broken out device elements. The bitfields are used to keep
      the overall value small enough to fit in a float accurately,
      which makes it possible to transport over the MAVLink
      parameter protocol without loss of information.
     */
    struct DeviceStructure 
	{
        enum BusType bus_type : 3;
        uint8_t bus: 5;    // which instance of the bus type
        uint8_t address;   // address on the bus (eg. I2C address)
        uint8_t devtype;   // device class specific device type
    };

    union DeviceId
	{
        struct DeviceStructure devid_s;
        uint32_t devid;
    };

    union DeviceId _bus_id;

    // set device address (eg. i2c bus address or spi CS)
    void set_device_address(uint8_t address)
	{
        _bus_id.devid_s.address = address;
    }

    // set device bus number
    void set_device_bus(uint8_t bus)
	{
        _bus_id.devid_s.bus = bus;
    }

private:
    // checked registers
    struct checkreg 
	{
        uint8_t regnum;
        uint8_t value;
    };
	
    struct 
	{
        uint8_t n_allocated;
        uint8_t n_set;
        uint8_t next;
        uint8_t frequency;
        uint8_t counter;
        struct checkreg *regs;
    } _checked;
};
