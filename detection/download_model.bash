mkdir -p data

if [ -f ./data/human_vino.xml ];
then
    echo "File human_vino.xml already exists, skipping"
else
    wget --no-check-certificate https://download.01.org/opencv/2021/openvinotoolkit/2021.2/open_model_zoo/models_bin/3/person-detection-0200/FP16/person-detection-0200.xml -O ./data/human_vino.xml
fi

if [ -f ./data/human_vino.bin ];
then
    echo "File human_vino.bin already exists, skipping"
else
    wget --no-check-certificate https://download.01.org/opencv/2021/openvinotoolkit/2021.2/open_model_zoo/models_bin/3/person-detection-0200/FP16/person-detection-0200.bin -O ./data/human_vino.bin
fi
