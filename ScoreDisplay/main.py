from time import sleep
from pySerialTransfer import pySerialTransfer as txfer

from tkinter import *
from math import *

main = Tk()
main.title('CALCULATOR')

font_height = 80
main.geometry('500x100')
Label(main, text="Hi Dino:", font=("default", font_height)).grid(row=0)
Label(main, text="Lo Dino:", font=("default", font_height)).grid(row=1)
Label(main, text="Green:", font=("default", font_height)).grid(row=2)
Label(main, text="Orange:", font=("default", font_height)).grid(row=3)

count_0_entry = Entry(main, font=("default", font_height))
count_1_entry = Entry(main, font=("default", font_height))
count_2_entry = Entry(main, font=("default", font_height))
count_3_entry = Entry(main, font=("default", font_height))

# offset_0 = 0
# offset_1 = 0
# offset_2 = 0
# offset_3 = 0
#
# def inc_off_0():
#     global offset_0
#     offset_0 += 1
#
# def dec_off_0():
#     global offset_0
#     offset_0 += 1
#
# Button(main, text='+', command=inc_off_0).grid(row=0, column=2, sticky=W)
# Button(main, text='-', command=add).grid(row=0, column=3, sticky=W,)
# Button(main, text='', command=sub).grid(row=0, column=4, sticky=W)

count_0_entry.grid(row=0, column=1)
count_1_entry.grid(row=1, column=1)
count_2_entry.grid(row=2, column=1)
count_3_entry.grid(row=3, column=1)

# mainloop()

count_0 = 0
count_1 = 0
count_2 = 0
count_3 = 0

# if __name__ == '__main__':
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

            count_0_entry.delete(0, END)
            count_0_entry.insert(0, count_0 * 10)
            count_1_entry.delete(0, END)
            count_1_entry.insert(0, count_1 * 10)
            count_2_entry.delete(0, END)
            count_2_entry.insert(0, count_2 * 5)
            count_3_entry.delete(0, END)
            count_3_entry.insert(0, count_3 * 5)

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
        main.update()


except KeyboardInterrupt:
    link.close()
