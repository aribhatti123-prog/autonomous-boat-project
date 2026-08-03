//Created and tested by Adam Fielding

#include <Arduino.h>

#define IMUSerial Serial2

#define PKT_BEGIN 0x49
#define PKT_END   0x4D
#define MAX_LEN   73

//Creates a class for the three vector
class Vec3 {
public:
    float x = 0, y = 0, z = 0;
};

//This class is used to store and print the IMUData recieved
class IMUData {
public:
    Vec3 acc;
    Vec3 gyro;
    Vec3 mag;
    Vec3 angle; // roll, pitch, yaw
    double heading;

    void print()
    {
        Serial.print("DIR");
        Serial.print(heading);
        Serial.print(" | ACC: ");
        Serial.print(acc.x); Serial.print(", ");
        Serial.print(acc.y); Serial.print(", ");
        Serial.print(acc.z);

        Serial.print(" | GYRO: ");
        Serial.print(gyro.x); Serial.print(", ");
        Serial.print(gyro.y); Serial.print(", ");
        Serial.print(gyro.z);

        Serial.print(" | MAG: ");
        Serial.print(mag.x); Serial.print(", ");
        Serial.print(mag.y); Serial.print(", ");
        Serial.print(mag.z);

        Serial.print(" | ANG: ");
        Serial.print(angle.x); Serial.print(", ");
        Serial.print(angle.y); Serial.print(", ");
        Serial.println(angle.z);
        
    }
};

//Important class, contains all the stuff to actually run the IMU
class IMUDriver {
private:
    //Temp array for data storage
    uint8_t buf[128];
    //Index tracking where in the buffer you are
    int idx = 0;
    //Used in the state machine to say what part of the packet is being read
    int state = 0;
    //length of payload, read from IMU
    int len = 0;
    //Checksum, ensures there's a valid packet
    uint8_t cs = 0;
    float magOffsetX = 37.09f;
    float magOffsetY = 1.51f;
    float magOffsetZ = -70.09f;
    double heading = 0;

    // scale constants (from IMU.uart.py)
    const float scaleAccel = 0.00478515625;
    const float scaleGyro  = 0.06103515625;
    const float scaleMag   = 0.15106201171875;
    const float scaleAngle = 0.0054931640625;

    //rd16() function reads the 8 bit stream as 16 bit endian which is what the IMU uses. 
    static int16_t rd16(uint8_t *b, int i)
    {
        return (int16_t)(b[i] | (b[i + 1] << 8));
    }
    //Finds the header byte
    int find0x11(uint8_t *d, int n)
    {
        for (int i = 0; i < n; i++)
            if (d[i] == 0x11) return i;
        return -1;
    }

   //Decoding the IMU stream
    void decode(uint8_t *d, int n)
    {
    //First finds the header
        int start = find0x11(d, n);
        if (start < 0) return;
    //Then finds the control byte, this is used to say what sensor reading you are looking at. e.g 0x0002 is accel (with grav)
        int ctl = (d[start + 2] << 8) | d[start + 1];
    //Sets the data pointer, +7 because that is the size of the header byte + timestamp + everything else before the actual sensor data
        int L = start + 7;
    //Next part self explanatory, checks ctl, if it's one we want, store and scale the data then move pointer. If not move pointer to next valid point.
        // --- ACC (no gravity) skip ---
        if (ctl & 0x0001)
        {
            L += 6;
        }

        // --- ACC (with gravity) ---
        if (ctl & 0x0002)
        {
            data.acc.x = rd16(d, L) * scaleAccel; L += 2;
            data.acc.y = rd16(d, L) * scaleAccel; L += 2;
            data.acc.z = rd16(d, L) * scaleAccel; L += 2;
        }

        // --- GYRO ---
        if (ctl & 0x0004)
        {
            data.gyro.x = rd16(d, L) * scaleGyro; L += 2;
            data.gyro.y = rd16(d, L) * scaleGyro; L += 2;
            data.gyro.z = rd16(d, L) * scaleGyro; L += 2;
        }

        // --- MAG ---
        if (ctl & 0x0008)
        {
            //Scales raw mag data as before
            float RawX = rd16(d, L) * scaleMag; L += 2;
            float RawY = rd16(d, L) * scaleMag; L += 2;
            float RawZ = rd16(d, L) * scaleMag; L += 2;
            //Actual outputted data is the scaled data - the offset
            data.mag.x = RawX - magOffsetX;
            data.mag.y = RawY - magOffsetY;
            data.mag.z = RawZ - magOffsetZ;
        }

        // --- TEMP + BARO (skip) ---
        if (ctl & 0x0010)
        {
            L += 2;
            L += 3;
            L += 3;
        }

        // --- QUATERNION (skip) ---
        if (ctl & 0x0020)
        {
            L += 8;
        }

        // --- ANGLES ---
        if (ctl & 0x0040)
        {
            data.angle.x = rd16(d, L) * scaleAngle; L += 2;
            data.angle.y = rd16(d, L) * scaleAngle; L += 2;
            data.angle.z = rd16(d, L) * scaleAngle; L += 2;
        }
        //Once all data has been recieved (currently just accel, gyro, mag and euler angles), sets new data flag to true so it can be read :)
        newData = true;
        //Computes the heading by taking the vector cross product of the x and y data points and multiplying by 180/pi
        double heading = atan2(data.mag.y, data.mag.x) * 180.0f / PI;
        //just some nice little correction :)
        if (heading < 0)
        heading += 360.0f;
        //Creates the data.heading value to be called to the print function 
        data.heading = heading;
    }

public:
    IMUData data;
    bool newData = false;
    //Processing the packets, basically state just tells the code where in the byte stream it is. Starting at the begin packet and ending at the end packet
    void process(uint8_t b)
    {
        cs += b;

        switch (state)
        {
        case 0:
            if (b == PKT_BEGIN)
            {
                idx = 0;
                buf[idx++] = b;
                cs = 0;
                state = 1;
            }
            break;

        case 1:
            buf[idx++] = b;
            state = 2;
            break;

        case 2:
            buf[idx++] = b;
            if (b == 0 || b > MAX_LEN)
                state = 0;
            else
            {
                len = b;
                state = 3;
            }
            break;

        case 3:
            buf[idx++] = b;
            if (idx >= len + 3)
                state = 4;
            break;

        case 4:
            cs -= b;
            if ((cs & 0xFF) == b)
            {
                buf[idx++] = b;
                state = 5;
            }
            else state = 0;
            break;

        case 5:
            state = 0;
            if (b == PKT_END)
            {
                buf[idx++] = b;
                decode(buf, idx);
            }
            break;
        }
    }

    //Function for sending a command to the IMU, currently sends bytes that tell the IMU to send and stream data
    void sendCmd(uint8_t *Pointer, uint8_t DataLength)
    {
        uint8_t tx[64];
        int idx = 0;

        for (int i = 0; i < 46; i++) tx[idx++] = 0;
        //Framing the data to be sent to the IMU, this is how it expects to receive stuff
        tx[idx++] = 0x00;
        tx[idx++] = 0xFF;
        tx[idx++] = 0x00;
        tx[idx++] = 0xFF;
        //Sending the Begin Command
        tx[idx++] = PKT_BEGIN;
        tx[idx++] = 0xFF;
        tx[idx++] = DataLength;

        for (int i = 0; i < DataLength; i++)
            tx[idx++] = Pointer[i];

        uint8_t cs = 0;
        for (int i = 51; i < 51 + DataLength + 2; i++)
            cs += tx[i];

        tx[idx++] = cs;
        tx[idx++] = PKT_END;
        //Writing all this to the IMUSerial
        IMUSerial.write(tx, idx);
    }

    /* =========================
       INITIALISE IMU
       ========================= */
    //Using the sendCmd line and the parameters to initialise the imu ready for streaming data
    void begin()
    {
        IMUSerial.begin(115200);
        delay(1000);

        uint16_t tag = 0x7F;

        uint8_t params[11] = {
            0x12, 5, 255, 0,
            0,
            100, 1, 3, 5,
            (uint8_t)(tag & 0xFF),
            (uint8_t)(tag >> 8)
        };

        sendCmd(params, 11);
        delay(200);

        uint8_t wake[] = {0x03};
        sendCmd(wake, 1);
        delay(200);

        uint8_t enable[] = {0x19};
        sendCmd(enable, 1);

        Serial.println("IMU INITIALISED");
    }
};
