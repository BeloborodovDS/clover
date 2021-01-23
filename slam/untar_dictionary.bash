if [ -f ./submodules/ORB_SLAM3/Vocabulary/ORBvoc.txt ];
then
    echo "Vocabulary already exists, skipping"
else
    tar -xf ./submodules/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz -C ./submodules/ORB_SLAM3/Vocabulary
fi
