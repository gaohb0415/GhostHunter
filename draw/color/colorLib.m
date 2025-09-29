function colorLib
% 常用绘图配色
% 作者: 刘涵凯
% 更新: 2023-12-18

% parula默认配色
colorBlue = [0 0.4470 0.7410];
colorOrange = [0.8500 0.3250 0.0980];
colorYellow = [0.9290 0.6940 0.1250];
colorPurple = [0.4940 0.1840 0.5560];
colorGreen = [0.4660 0.6740 0.1880];
colorCyan = [0.3010 0.7450 0.9330];
colorRed = [0.6350 0.0780 0.1840];
colorSetParula = [colorBlue; colorOrange; colorYellow; colorPurple; colorGreen; colorCyan; colorRed];

% Jama配色
colorGrayJama = [0.216, 0.306, 0.333];
colorYellowJama = [0.831, 0.561, 0.267];
colorBlueJama = [0, 0.631, 0.835];
colorRedJama = [0.698, 0.278, 0.271];
colorGreenJama = [0.475, 0.686, 0.592];
colorPurpleJama = [0.416, 0.396, 0.60];
colorBrownJama = [0.502, 0.475, 0.42];
colorSetJama = [colorGrayJama; colorYellowJama; colorBlueJama; colorRedJama; colorGreenJama; colorPurpleJama; colorBrownJama];

% Morandi配色
colorMorandiBlue = [118, 134, 146] / 255;
colorMorandiRed = [156, 88, 62] / 255;
colorMorandiYellow = [201, 158, 78] / 255;
colorMorandiGreen = [128, 137, 122] / 255;
colorMorandiPurple = [127, 115, 132] / 255;
colorMorandiBrown = [122, 106, 91] / 255;
colorMorandiPink = [202, 195, 212] / 255;
colorMorandiBlue2 = [153, 164, 188] / 255;
colorMorandiYellow2 = [238, 234, 193] / 255;
colorMorandiBlueDraw = [118, 134, 146] / 255;
colorMorandiPurpleDraw = [137, 120, 124] / 255;
colorMorandiBrownDraw = [177, 138, 106] / 255;
colorSetMorandi = [colorMorandiBlue; colorMorandiRed; colorMorandiYellow; ...
    colorMorandiGreen; colorMorandiPurple; colorMorandiBrown; colorMorandiPink; ...
    colorMorandiBlue2; colorMorandiYellow2; colorMorandiBlueDraw; ...
    colorMorandiPurpleDraw; colorMorandiBrownDraw];

% 使用过的其他颜色
colorBlue2 = [0.3098, 0.5059, 0.7412];
colorRed2 = [0.7529, 0.3137, 0.2980];
colorGreen2 = [0.6078, 0.7333, 0.3490];
colorPurple2 = [0.5020, 0.3922, 0.6353];
colorBlue3 = [0.07,0.62,1.00];
colorBlue4 = [0.1020, 0.4353, 0.8745];
colorGray = [0.5, 0.5, 0.5];
colorGray2 = [0.8, 0.8, 0.8];
colorOthers = [colorBlue2; colorRed2; colorGreen2; colorPurple2; colorBlue3; colorBlue4; ...
    colorGray; colorGray2];

% 将所有颜色组成序列
colorSet = [colorSetParula; colorSetJama; colorSetMorandi; colorOthers];
save .\draw\color\colorLib.mat