rm test.hex;
rm test.bin;
touch test.hex;
touch test.bin;

INPUT_BASE=10
OUTPUT_BASE=16

for  ((x=0; x < 255; x+=4))
do
    STRING_A=$(printf "%02x" $x)
    STRING_B=$(printf "%02x" $(($x+1)))
    STRING_C=$(printf "%02x" $(($x+2)))
    STRING_D=$(printf "%02x" $(($x+3)))

    STRING_0=${STRING_D:0:2}
    STRING_8=${STRING_C:0:2}
    STRING_16=${STRING_B:0:2}
    STRING_24=${STRING_A:0:2}

    echo -n $STRING_24 $STRING_16 $STRING_8 $STRING_0" ">> test.hex
    echo $STRING_24 $STRING_16 $STRING_8 $STRING_0 $x

done;

xxd -r -ps test.hex test.bin

