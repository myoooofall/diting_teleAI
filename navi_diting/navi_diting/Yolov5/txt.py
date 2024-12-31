from glob import glob

train_dir = '/storage/A_Yolov5/Product/images/train'
val_dir = '/storage/A_Yolov5/Product/images/val'

train_imgs = glob(train_dir + '/*.jpg')
val_imgs = glob(val_dir + '/*.jpg')

with open('/storage/A_Yolov5/Product/train919.txt', 'w') as f:
    for img in train_imgs:
        f.write(img + '\n')
    f.close()

with open('/storage/A_Yolov5/Product/val919.txt', 'w') as f:
    for img in val_imgs:
        f.write(img + '\n')
    f.close()