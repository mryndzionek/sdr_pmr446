#!/bin/sh

if [ ! -e appimagetool-x86_64.AppImage ]
then
    wget https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage
    chmod +x appimagetool-x86_64.AppImage
fi

rm -Rf AppDir
mkdir AppDir
cd AppDir

mkdir lib
mkdir -p lib/SoapySDR/modules0.8

LIBS=`find ../local -name *.so*`
for l in $LIBS
do
    cp -a $l lib
done

mv lib/librtlsdrSupport.so lib/SoapySDR/modules0.8
rm lib/_SoapySDR.so

mkdir bin
cp ../build/sdr_pmr446 bin

cp ../sdr_pmr446.svg .
cp ../AppRun .

echo "[Desktop Entry]" > sdr_pmr446.desktop
echo "Type=Application" >> sdr_pmr446.desktop
echo "Name=SDR PMR446" >> sdr_pmr446.desktop
echo "Comment=SDR PMR446 scanner application" >> sdr_pmr446.desktop
echo "Exec=sdr_pmr446" >> sdr_pmr446.desktop
echo "Icon=sdr_pmr446" >> sdr_pmr446.desktop
echo "Categories=Utility;" >> sdr_pmr446.desktop
echo "Terminal=true" >> sdr_pmr446.desktop

cd ..
./appimagetool-x86_64.AppImage AppDir sdr_pmr446.AppImage

