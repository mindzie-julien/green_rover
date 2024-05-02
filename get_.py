from smbus2 import SMBus, i2c_msg
import time
import json
import computer as pc

I2C_SLAVE_ADDR = 0x8
ASK_FOR_LENGTH = 0x0
ASK_FOR_DATA = 0x1
I2C_LENGTH_LIMIT = 32
SLEEP_TIME = 0

with SMBus(1) as bus:

    time.sleep(SLEEP_TIME)
    def ask_data_lenght():
        # Ask for length response
        write = i2c_msg.write(I2C_SLAVE_ADDR, [ASK_FOR_LENGTH])
        bus.i2c_rdwr(write)

        time.sleep(SLEEP_TIME)

        # Answer
        read = i2c_msg.read(I2C_SLAVE_ADDR, 1)
        bus.i2c_rdwr(read)
        responseLength = list(read)[0]

        return responseLength

    time.sleep(SLEEP_TIME)


    def send_json(data):
        # Envoie des données au format JSON à l'esclave
        json_data = json.dumps(data)

        # Envoi du JSON à l'Arduino par blocs
        for i in range(0, len(json_data), I2C_LENGTH_LIMIT):
            write_data = [ord(c) for c in json_data[i:i+I2C_LENGTH_LIMIT]]
            write = i2c_msg.write(I2C_SLAVE_ADDR, [ASK_FOR_DATA] + write_data)
            bus.i2c_rdwr(write)


    def receive_json():
    # Ask for data reponse
        responseLength = ask_data_lenght()
        write = i2c_msg.write(I2C_SLAVE_ADDR, [ASK_FOR_DATA])
        bus.i2c_rdwr(write)

        time.sleep(SLEEP_TIME)

        response = str()

        # Answer: Iterate over I2C_LENGTH_LIMIT bytes blocks, plus last [0,I2C_LENGTH_LIMIT] block
        for responseIndex in range(0, (responseLength // I2C_LENGTH_LIMIT) + 1, 1):
            read = i2c_msg.read( \
                I2C_SLAVE_ADDR, \
                I2C_LENGTH_LIMIT if (responseIndex != (responseLength // I2C_LENGTH_LIMIT)) else (responseLength % I2C_LENGTH_LIMIT))
            bus.i2c_rdwr(read)
            response +=  "".join([chr(i) for i in list(read)])

        parsedJson = json.loads(response)
        return parsedJson
#  ###################################################

    lenght = ask_data_lenght()
    response = receive_json()
    pc.writeData("feedback.json", response)
    
 

