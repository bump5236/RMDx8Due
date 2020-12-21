
#include <Arduino.h>
#include "RMDx8Due.h"

// constructor
RMDx8Due::RMDx8Due(CAN_COMMON &CAN, const uint16_t motor_addr) 
    :_CAN(CAN){
        MOTOR_ADDRESS = motor_addr;
        CAN_FRAME rxMsg, txMsg;  //TODO
    }


void RMDx8Due::canSetup() {
    while (0 != _CAN.begin(CAN_BPS_1000K)) {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}


void RMDx8Due::readPID() {
    cmd_buf[0] = 0x30;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;
    
    writeCmd(cmd_buf);  // Send message
    readBuf(cmd_buf);  // Read message
    if (isRead) {
        posKp = reply_buf[2];
        posKi = reply_buf[3];
        velKp = reply_buf[4];
        velKi = reply_buf[5];
        curKp = reply_buf[6];
        curKi = reply_buf[7];
    }
}


void RMDx8Due::writePID(uint8_t anglePidKp, uint8_t anglePidKi, uint8_t speedPidKp, uint8_t speedPidKi, uint8_t iqPidKp, uint8_t iqPidKi) {
    cmd_buf[0] = 0x31;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = anglePidKp;
    cmd_buf[3] = anglePidKi;
    cmd_buf[4] = speedPidKp;
    cmd_buf[5] = speedPidKi;
    cmd_buf[6] = iqPidKp;
    cmd_buf[7] = iqPidKi;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);
}


void RMDx8Due::writeEncoderOffset(uint16_t offset) {
    cmd_buf[0] = 0x91;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = offset & 0xFF;
    cmd_buf[7] = (offset >> 8) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);
}


void RMDx8Due::readPosition() {
    cmd_buf[0] = 0x92;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    if (isRead) {
        pos_u32t = ((uint32_t)reply_buf[4] << 24) + ((uint32_t)reply_buf[3] << 16) + ((uint32_t)reply_buf[2] << 8) + reply_buf[1];

        if (pos_u32t > 2147483648) {
            present_position = pos_u32t - 4294967296;
        }
        else {
            present_position = pos_u32t;
        }
    }
}


void RMDx8Due::clearState() {
    cmd_buf[0] = 0x80;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    writeCmd(cmd_buf);
    readBuf(cmd_buf);
}


/**
 * current is int16_t type, the value range:-2000~2000, corresponding to the actual torque current range -12.5A ~ 12.5A.
 * (the bus current and the actual torque of motor vary with different motors)
 */
void RMDx8Due::writeCurrent(int16_t current) {
    cmd_buf[0] = 0xA1;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = current & 0xFF;
    cmd_buf[5] = (current >> 8) & 0xFF;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    if (isRead) {
        temperature = reply_buf[1];
        present_current = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
        present_velocity = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
        encoder_pos = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    }
}


/**
 * velocity is int32_t type, which corresponds to the actual speed of 0.01 dps/LSB.
 */
void RMDx8Due::writeVelocity(int32_t velocity) {
    cmd_buf[0] = 0xA2;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = velocity & 0xFF;
    cmd_buf[5] = (velocity >> 8) & 0xFF;
    cmd_buf[6] = (velocity >> 16) & 0xFF;
    cmd_buf[7] = (velocity >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    if (isRead) {
        temperature = reply_buf[1];
        present_current = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
        present_velocity = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
        encoder_pos = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    }
}


/**
 * # Position control command 1, multi turns
 * position is int32_t type, and the actual position is 0.01 degree/LSB, 36000 represents 360°.
 * The motor rotation direction is determined by the difference between the target position and the current position.
 */
void RMDx8Due::writePosition(int32_t position) {
    cmd_buf[0] = 0xA3;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = position & 0xFF;
    cmd_buf[5] = (position >> 8) & 0xFF;
    cmd_buf[6] = (position >> 16) & 0xFF;
    cmd_buf[7] = (position >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    if (isRead) {
        temperature = reply_buf[1];
        present_current = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
        present_velocity = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
        encoder_pos = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    }
}


/**
 * # Position control command 2, multi turns
 * In addition to Position control command 1, the following functions have been added.
 * The control value max_speed limits the maximum speed at which the motor rotates, uint16_t type, corresponding to the actual speed of 1 dps/LSB.
 */
void RMDx8Due::writePosition(int32_t position, uint16_t max_speed) {
    cmd_buf[0] = 0xA4;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = max_speed & 0xFF;
    cmd_buf[3] = (max_speed >> 8) & 0xFF;
    cmd_buf[4] = position & 0xFF;
    cmd_buf[5] = (position >> 8) & 0xFF;
    cmd_buf[6] = (position >> 16) & 0xFF;
    cmd_buf[7] = (position >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    if (isRead) {
        temperature = reply_buf[1];
        present_current = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
        present_velocity = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
        encoder_pos = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    }
}


/**
 * # Position control command 3, single turn
 * position is uint16_t type, the value range is 0~35999, and the actual position is 0.01 degree/LSB, the actual angle range is 0°~359.99°.
 * The control value spin_direction sets the direction in which the motor rotates, which is uint8_t type, 0x00 for CW and 0x01 for CCW.
 */
void RMDx8Due::writePosition(uint16_t position, uint8_t spin_direction) {
    cmd_buf[0] = 0xA5;
    cmd_buf[1] = spin_direction;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = position & 0xFF;
    cmd_buf[5] = (position >> 8) & 0xFF;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    if (isRead) {
        temperature = reply_buf[1];
        present_current = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
        present_velocity = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
        encoder_pos = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    }
}


/** 
 * # Position control command 4, single turn
 * position is uint16_t type, the value range is 0~35999, and the actual position is 0.01 degree/LSB, the actual angle range is 0°~359.99°.
 * The control value max_speed limits the maximum speed at which the motor rotates, uint16_t type, corresponding to the actual speed of 1 dps/LSB.
 * The control value spin_direction sets the direction in which the motor rotates, which is uint8_t type, 0x00 for CW and 0x01 for CCW.
 */
void RMDx8Due::writePosition(uint16_t position, uint16_t max_speed, uint8_t spin_direction) {
    cmd_buf[0] = 0xA6;
    cmd_buf[1] = spin_direction;
    cmd_buf[2] = max_speed & 0xFF;
    cmd_buf[3] = (max_speed >> 8) & 0xFF;
    cmd_buf[4] = position & 0xFF;
    cmd_buf[5] = (position >> 8) & 0xFF;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    if (isRead) {
        temperature = reply_buf[1];
        present_current = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
        present_velocity = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
        encoder_pos = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
    }
}

// General function
void RMDx8Due::serialWriteTerminator() {
    Serial.write(13);
    Serial.write(10);
}


// Private
void RMDx8Due::readBuf(unsigned char *buf) {
    isRead = 0;

    if (_CAN.available() > 0) {
        _CAN.read(rxMsg);

        if (rxMsg.data.byte[0] == buf[0]) {
            reply_buf[0] = rxMsg.data.byte[0];
            reply_buf[1] = rxMsg.data.byte[1];
            reply_buf[2] = rxMsg.data.byte[2];
            reply_buf[3] = rxMsg.data.byte[3];
            reply_buf[4] = rxMsg.data.byte[4];
            reply_buf[5] = rxMsg.data.byte[5];
            reply_buf[6] = rxMsg.data.byte[6];
            reply_buf[7] = rxMsg.data.byte[7];

            isRead = 1;
        }    
    }
}


void RMDx8Due::writeCmd(unsigned char *buf) {
    txMsg.data.byte[0] = buf[0];
	txMsg.data.byte[1] = buf[1];
	txMsg.data.byte[2] = buf[2];
	txMsg.data.byte[3] = buf[3];
	txMsg.data.byte[4] = buf[4];
	txMsg.data.byte[5] = buf[5];
	txMsg.data.byte[6] = buf[6];
	txMsg.data.byte[7] = buf[7];

    // CAN通信で送る
    unsigned char sendState = _CAN.sendFrame(txMsg);
    if (sendState != false) {
        Serial.println("Error Sending Message...");
        Serial.println(sendState);
    }
}
