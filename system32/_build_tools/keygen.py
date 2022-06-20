#!/usr/bin/env python3
import sys, os
from Crypto.Random import get_random_bytes

class DirectoryNotFoundError(Exception):
    pass

def program_quit(arg):
    if(arg == 0):
        print("\n== Finished!")
    else:
        print("\n== Terminating")
    exit(arg)

if __name__ == '__main__':
              
                                      
    print("== ╦╔═╔═╗╦ ╦╔═╗╔═╗╔╗╔\n"+
          "== ╠╩╗║╣ ╚╦╝║ ╦║╣ ║║║\n"+
          "== ╩ ╩╚═╝ ╩ ╚═╝╚═╝╝╚╝")
    print("== AES256 key file generator")
    print("== (C) 2022 j0ule, All Rights Reserved\n")

    # Get keys directory from cmd line
    try:
        # Was any directory passed?
        try:
            keys_directory = sys.argv[1]
        except:
            print("== Error: no project directory!")
            raise FileNotFoundError
    except FileNotFoundError:
        # Uh oh, no files passed
        print("== Proper usage is {} <keys directory> <sources_directory> <project_name>".format(sys.argv[0]))
        program_quit(1)


    # Get project directory from cmd line
    try:
        # Was any directory passed?
        try:
            project_directory = sys.argv[2]
        except:
            print("== Error: no sources directory!")
            raise FileNotFoundError
    except FileNotFoundError:
        # Uh oh, no files passed
        print("== Proper usage is {} <keys directory> <sources_directory> <project_name>".format(sys.argv[0]))
        program_quit(1)


    # Get project name from cmd line
    try:
        # Was any name passed?
        try:
            project_name = sys.argv[3]
        except:
            print("== Error: no project!")
            raise FileNotFoundError
    except FileNotFoundError:
        # Uh oh, no files passed
        print("== Proper usage is {} <keys directory> <sources_directory> <project_name>".format(sys.argv[0]))
        program_quit(1)


    # Try to see if there is a key in the keys directory already
    output_file_path = keys_directory + '/' + project_name + '/bootkey.h'

    try:
        if(not os.path.exists(project_directory + '/' + project_name)):
            raise DirectoryNotFoundError

        if os.path.exists(output_file_path):
            raise FileExistsError
        else:
            raise FileNotFoundError
        
    except FileNotFoundError:
        # If no file found it is not actually an error
        pass

    except DirectoryNotFoundError:
        # If there was no directory this is not a valid source!
        print("== Error: Project source directory does not exist!")
        program_quit(1)

    except FileExistsError:
        # If there was a file, we should not over-write the old key!
        print("== Warning: Key file already exists, will not overwrite.")
        program_quit(0)

    # Create key
    aes_256_key = get_random_bytes(32)
    aes_256_key_cpp_style = ', '.join('0x{:02X}'.format(byte) for byte in aes_256_key)

    # Create keyfile
    keyfile_string  = "#ifndef __KEYFILE_H__\n"
    keyfile_string += "    #define __KEYFILE_H__\n"
    keyfile_string += "    \n"
    keyfile_string += "    #define BOOT_KEY {{ {} }}\n".format(aes_256_key_cpp_style)
    keyfile_string += "    \n"
    keyfile_string += "#endif"

    keyfile_bytes = keyfile_string.encode('utf-8')


    # Write output file
    print("\nWriting output file \"{}\"".format(output_file_path))
    try:
        os.makedirs(os.path.dirname(output_file_path), exist_ok=True)
        output_file = open(output_file_path, 'wb')
        written_bytes = output_file.write(keyfile_bytes)
        output_file.close()
    except:
        # Uh oh, permissions error?
        print("== Error: could not write output file!")
        program_quit(1)

    # Finish program
    print('\tDone! \n\tFile length {} bytes.'.format(written_bytes))
    program_quit(0)
