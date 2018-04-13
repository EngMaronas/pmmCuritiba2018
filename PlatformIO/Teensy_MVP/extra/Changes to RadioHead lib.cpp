/* By Henrique Bruno, UFRJ Minerva Rockets. Send an array of pointers to 4 bytes variables. Do typecast on the variables before, to uint8_t.
    float floatVar;
    uint32_t uint32Var;
    uint8_t *array[SIZE] =
    {
        (uint8_t*) & floatVar,
        (uint8_t*) & uint32Var
    }
    sendArrayOfPointersOf4Bytes(array, SIZE)
*/
bool RH_RF95::sendArrayOfPointersOf4Bytes(const uint8_t** data, uint8_t number4BytesVariables)
{

    waitPacketSent(); // Make sure we dont interrupt an outgoing message
    setModeIdle();

    // Position at the beginning of the FIFO
    spiWrite(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
    // The headers
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderTo);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFrom);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderId);
    spiWrite(RH_RF95_REG_00_FIFO, _txHeaderFlags);
    // The message data
    spiBurstWriteArrayOfPointersOf4Bytes(RH_RF95_REG_00_FIFO, data, number4BytesVariables);
    spiWrite(RH_RF95_REG_22_PAYLOAD_LENGTH, (number4BytesVariables * 4) + RH_RF95_HEADER_LEN);

    setModeTx(); // Start the transmitter
    // when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
    return true;
}

/* By Henrique Bruno - UFRJ Minerva Rockets*/
uint8_t RHSPIDriver::spiBurstWriteArrayOfPointersOf4Bytes(uint8_t reg, const uint8_t** src, uint8_t number4BytesVariables)
{
    uint8_t status = 0;
    ATOMIC_BLOCK_START;
#if defined(SPI_HAS_TRANSACTION)
    SPI.beginTransaction(_spi._settings);
#endif
    digitalWrite(_slaveSelectPin, LOW);
    status = _spi.transfer(reg | RH_SPI_WRITE_MASK); // Send the start address with the write mask on
    while (number4BytesVariables--)
    {
        _spi.transfer(**src++);
        _spi.transfer(**src++);
        _spi.transfer(**src++);
        _spi.transfer(**src++);
    }
    digitalWrite(_slaveSelectPin, HIGH);
#if defined(SPI_HAS_TRANSACTION)
    SPI.endTransaction();
#endif
    ATOMIC_BLOCK_END;
    return status;
}
