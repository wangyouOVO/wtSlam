#include"showVideo.hpp"

int main(int argc,char **argv){
    std::string fileName("../videos/home.mp4");
    wtSlam::ShowVideoUtil lxz(fileName);
    lxz.showVideo();
    return 0;
}