import sys, os
if(len(sys.argv) < 2):
    print('please input video file from command line, for example: python3 run_test_video.py /xxxpath/xxx.avi')
    exit(1)
video_file = sys.argv[1]
os.system('python3 detectionExample/Main.py --video ' + video_file + ' --graph 20190116093312_TinyYoloV3NCS.graph_288')
