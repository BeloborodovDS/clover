mkdir -p data

if [ -f ./data/human_vino.xml ];
then
    echo "File human_vino.xml already exists, skipping"
else
    wget --no-check-certificate https://download.01.org/openvinotoolkit/2018_R4/open_model_zoo/pedestrian-detection-adas-0002/FP16/pedestrian-detection-adas-0002.xml -O ./data/human_vino.xml
fi

if [ -f ./data/human_vino.bin ];
then
    echo "File human_vino.bin already exists, skipping"
else
    wget --no-check-certificate https://download.01.org/openvinotoolkit/2018_R4/open_model_zoo/pedestrian-detection-adas-0002/FP16/pedestrian-detection-adas-0002.bin -O ./data/human_vino.bin
fi
