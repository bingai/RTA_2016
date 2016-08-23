function [imagePoints1,imagePoints2,boardSize,imagesUsed1, imagesUsed2] = detectCornersStereo(directory1,directory2)  
% detects checkerboard for all files in provided directory,
% saves output as .csv files
    [imagePoints1,~,imagesUsed1] = detectCornersInDirectoryNoSave(directory1);
    [imagePoints2,boardSize,imagesUsed2] = detectCornersInDirectoryNoSave(directory2);
    dlmwrite(strcat('imagePoints_',directory1, '.csv'), imagePoints1, ...
        'delimiter', ',', 'precision', 12);
    dlmwrite(strcat('imagePoints_',directory2, '.csv'), imagePoints2, ...
        'delimiter', ',', 'precision', 12);
    csvwrite(strcat('boardSize_','stereo', '.csv'),boardSize)
    csvwrite(strcat('imagesUsed_',directory1, '.csv'),imagesUsed1)
    csvwrite(strcat('imagesUsed_',directory2, '.csv'),imagesUsed2)
end