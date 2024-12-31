    
    save_count = 0
    try:
        while True and save_count < 60:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            rgb_image = np.asanyarray(color_frame.get_data())
            save_count += 1
                
            # 保存视频截图
            if save_count > 55:
                file_path = f"/home/jianghao/source/frame_{save_count}.jpg"
                
                cv.imwrite(file_path, rgb_image)
                # cv.imshow('my webcam', rgb_image) 
                # key = cv.waitKey(1)
                # if key & 0xFF == ord('q') or key == 27:
                #     cv.destroyAllWindows()
                #     break
    finally:
        pipeline.stop()  

    
    paths = glob("/home/jianghao/source/*jpg")