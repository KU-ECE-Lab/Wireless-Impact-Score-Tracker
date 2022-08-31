from time import sleep
from pySerialTransfer import pySerialTransfer as txfer


count_0 = 0
count_1 = 0
count_2 = 0
count_3 = 0

if __name__ == '__main__':
    try:
        link = txfer.SerialTransfer('COM12')

        link.open()
        sleep(5)

        while True:
            if link.available():
                recSize = 0

                count_0 = link.rx_obj(obj_type=int,
                                  start_pos=recSize,
                                  obj_byte_size=4)
                # recSize += len(count_0)
                recSize += 4
                count_1 = link.rx_obj(obj_type=int,
                                  start_pos=recSize,
                                  obj_byte_size=4)
                # recSize += len(count_1)
                recSize += 4
                count_2 = link.rx_obj(obj_type=int,
                                  start_pos=recSize,
                                  obj_byte_size=4)
                # recSize += len(count_2)
                recSize += 4
                count_3 = link.rx_obj(obj_type=int,
                                  start_pos=recSize,
                                  obj_byte_size=4)
                # recSize += len(count_3)
                recSize += 4

                print('C0: {} C1: {} C2: {} C3: {}'.format(count_0, count_1, count_2, count_3))

            elif link.status < 0:
                if link.status == txfer.CRC_ERROR:
                    print('ERROR: CRC_ERROR')
                elif link.status == txfer.PAYLOAD_ERROR:
                    print('ERROR: PAYLOAD_ERROR')
                elif link.status == txfer.STOP_BYTE_ERROR:
                    print('ERROR: STOP_BYTE_ERROR')
                else:
                    print('ERROR: {}'.format(link.status))


    except KeyboardInterrupt:
        link.close()
