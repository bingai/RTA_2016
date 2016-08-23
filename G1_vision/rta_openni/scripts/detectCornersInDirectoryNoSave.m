function [imagePoints,boardSize,imagesUsed, image_paths] = detectCornersInDirectoryNoSave(directory)  
% detects checkerboard for all files in provided directory,
% saves output as .csv files
    image_files = dir(strcat(directory, filesep,'*.png'));
    image_files = {image_files.name};
    image_paths = {};
    frame_numbers = {};
    digits_expr = '\d+';
    for i_file = 1:length(image_files)
        image_name = image_files(i_file);
        path = strcat(directory, filesep, image_name);
        n_frame_str = regexp(image_name,digits_expr,'match');
        n_frame_str = n_frame_str{1};
        n_frame = str2num(char(n_frame_str));
        image_paths = [image_paths, path];
        frame_numbers = [frame_numbers, n_frame];
    end
    imagesUsed = {};
    [imagePoints,boardSize,imagesUsedBool] = detectCheckerboardPoints(image_paths);
    for i_im = 1: length(frame_numbers)
        if(imagesUsedBool(i_im))
            imagesUsed = [imagesUsed, frame_numbers(i_im)];
        end
    end
end