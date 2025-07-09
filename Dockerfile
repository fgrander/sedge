FROM fizmath/gpu-opencv

WORKDIR /usr/src/app

RUN pip install requests --upgrade
RUN pip install numpy --upgrade

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

RUN apt-get update --allow-insecure-repositories && apt-get install -y ffmpeg
#RUN apt-get install libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio


CMD [ "python3", "./axis.py" ]