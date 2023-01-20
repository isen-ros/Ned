#!/usr/bin/env bash

mkdir bin
cd bin
wget -O- get.pharo.org/100 | bash
wget -O- get.pharo.org/vm100 | bash
./pharo Pharo.image eval --save "Metacello new
		repository: 'github://sbragagnolo/Sagan:main/src';
		baseline: 'Sagan';
		load."
cd ..
echo "#!/usr/bin/env bash
cd bin
./pharo-ui Pharo.image
cd ..
" > sagan.sh
chmod +x sagan.sh
