#!/usr/bin/env python3
import sys, os, array
from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes

#AES_KEY =   [0x60, 0x3d, 0xeb, 0x10, 0x15, 0xca, 0x71, 0xbe, 0x2b, 0x73, 0xae, 0xf0, 0x85, 0x7d, 0x77, 0x81,
#            0x1f, 0x35, 0x2c, 0x07, 0x3b, 0x61, 0x08, 0xd7, 0x2d, 0x98, 0x10, 0xa3, 0x09, 0x14, 0xdf, 0xf4]

class AESCipher:
    def __init__(self, aes_key, bootloader_size):
        self.key = bytes(aes_key)
        self.key_bits = 8 * self.key
        self.block_size = AES.block_size
        self.bootloader_size = bootloader_size

    # The encryption we use for the bootloader puts the IV at the beginning of the file
    # and then pads the application with 0xFF until we reach the beginning of application
    # space. The bootloader will not write these 0xFF's to flash (as doing so would destroy
    # the bootloader), but once we are in application space the flashing will commence.
    def encrypt(self, data):
        init_vector = bytearray(get_random_bytes(self.block_size))
        self.cipher = AES.new(self.key, AES.MODE_CBC, bytes(init_vector))

        # Pad data with bootloader size - IV size
        padded_data = init_vector + bytearray(get_random_bytes(self.bootloader_size - self.block_size)) + data
        padded_data_bytes = bytes(padded_data)

        return self.cipher.encrypt(padded_data_bytes)

    # Runs the same algortithm the bootloader runs
    def decrypt(self, data):
        init_vector = data[:self.block_size] 

        self.cipher = AES.new(self.key, AES.MODE_CBC, init_vector)
        plaintext = self.cipher.decrypt(data[self.block_size:])

        unpadded_plaintext = plaintext[(self.bootloader_size - self.block_size):]
        return unpadded_plaintext


def program_quit(arg):
    if(arg == 0):
        print("\n== Finished!")
    else:
        print("\n== Terminating")
    exit(arg)

if __name__ == '__main__':

    print("== ╦  ╔═╗╔═╗╦╔═╔╦╗╔═╗╦ ╦╔╗╔\n"+
          "== ║  ║ ║║  ╠╩╗ ║║║ ║║║║║║║\n"+
          "== ╩═╝╚═╝╚═╝╩ ╩═╩╝╚═╝╚╩╝╝╚╝")
    print("== AES256 arbitrary binary file encrypter")
    print("== (C) 2019 Adam Munich, All Rights Reserved\n")

    # Get input and output file path from command line
    try:
        # Was any input file passed?
        try:
            input_file_name = sys.argv[1]
        except:
            print("== Error: no input file!")
            raise FileNotFoundError
        
        # Was any output file passed?
        try:
            output_file_name = sys.argv[2]
        except:
            print("== Error: no output file!")
            raise FileNotFoundError

        # Was any bootkey file passed?
        try:
            bootkey_file_name = sys.argv[3]
        except:
            print("== Error: no key file!")
            raise FileNotFoundError

    except FileNotFoundError:
        # Uh oh, no files passed
        print("== Proper usage is {} <input file> <output file> <bootkey file>".format(sys.argv[0]))
        program_quit(1)


    # Get bootloader size from CMD line
    bootloader_size = 0
    try:
        bootloader_size = int(sys.argv[4], 0)
        print("Bootloader size specified to be {} bytes.".format(bootloader_size))
    except:
        print("No bootloader size specified, assuming {} bytes.".format(bootloader_size))


    # Try to open bootkey file
    try:
        bootkey_file = open(bootkey_file_name, 'r')
    except:
        # Uh oh, can't open input file
        print("== Error: could not open bootkey file!")
        program_quit(1)

    print("\nOpened key file \"{}\"".format(bootkey_file_name))
    
    # Try to extract the crypto key from key file
    try:
        bootkey_file_parts    = bootkey_file.read().split('BOOT_KEY')
        bootkey_file_key_half = bootkey_file_parts[1]
        bootkey_file_key_line = bootkey_file_key_half.split('\n')[0] 
        bootkey_file_key_list = bootkey_file_key_line.replace('{', '').replace('}', '').strip()
        bootkey_file_key_hexs = bootkey_file_key_list.split(', ')
        bootkey_file_key_ints = [int(x, 16) for x in bootkey_file_key_hexs]
        if(len(bootkey_file_key_ints) != 32):
            print("== Error: key is incorrect length!")
            raise FileNotFoundError
        bootkey_file_bytes = array.array('B', bootkey_file_key_ints).tobytes()
        AES_KEY = bootkey_file_bytes
        print("\tSuccessfully parsed key file.")
    except Exception as e:
        # Uh oh, can't parse key file
        print("== Error: invalid boot key file!")
        program_quit(1)


    # Try to open input file
    try:
        input_file = open(input_file_name, 'rb')
    except:
        # Uh oh, can't open input file
        print("== Error: could not open input file!")
        program_quit(1)

    print("\nOpened input binary file \"{}\"".format(input_file_name))

    # Read file into bytearray
    input_byte_array = bytearray(input_file.read())
    input_file.close()
    print("\tRead {} bytes.".format(len(input_byte_array)))

    # Test to see if file is modulo AES block size
    must_pad = False
    if(len(input_byte_array) % len(AES_KEY) != 0):
        print("\tBinary length not encryptable!")
        must_pad = True

    # Pad end of file with 0xFFif length of the file is not mod 16
    padding_length = 0
    while(must_pad is True and (len(input_byte_array) % len(AES_KEY)) != 0):
        input_byte_array += b'\xFF'
        padding_length += 1
    
    if(must_pad is True):
        print("\tPadded binary with {} bytes.".format(padding_length))
        encryptable_length = len(input_byte_array)
        print("\tNew binary length is {} bytes.".format(encryptable_length))


    # Encrypt file
    print("\nEncrypting...")
    ciphertext = AESCipher(AES_KEY, bootloader_size).encrypt(input_byte_array)
    encrypted_length = len(ciphertext)
    print("\tDone! \n\tCiphertext length is {} bytes.".format(encrypted_length))


    # Decrypt file
    print("\nDecrypting...")
    decrypted = AESCipher(AES_KEY, bootloader_size).decrypt(ciphertext)
    decrypted_length = len(decrypted)
    print("\tDone! \n\tDecrypted length is {} bytes.".format(decrypted_length))


    # Verifify decrypt matches plaintext
    print("\nVerifying...")
    if input_byte_array == bytearray(decrypted):
        print("\tDone! \n\tDecryption matches source.")
    else:
        print("== Error: decryption mismatches encryption!")
        program_quit(1)


    # Write output file
    print("\nWriting output file \"{}\"".format(output_file_name))
    try:
        os.makedirs(os.path.dirname(output_file_name), exist_ok=True)
        output_file = open(output_file_name, 'wb')
        written_bytes = output_file.write(ciphertext)
        output_file.close()
    except:
        # Uh oh, permissions error?
        print("== Error: could not write output file!")
        program_quit(1)


    # Finish program
    print('\tDone! \n\tFile length {} bytes.'.format(written_bytes))
    program_quit(0)
