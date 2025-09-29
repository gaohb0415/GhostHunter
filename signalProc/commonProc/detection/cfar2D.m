function [ptIdxRow, ptIdxCol, th] = cfar2D(value, cfarParam)
% 调用MATLAB phased库进行2D CFAR检测
% 输入:
% 1. value: 数值矩阵
% 2. cfarParam:
%     - .train: 训练单元, 单边
%     - .guard: 保护单元, 单边
%     - .pfa: 虚警概率
%     - .extraTh: 额外阈值
% 输出:
% 1. ptIdxRow: 点云的行索引
% 2. ptIdxCol: 点云的列索引
% 3. th: CFAR阈值


% 创建CFAR作用点范围
% 解决"边界问题"
% 2D CFAR与1D CFAR 本质原理基本相同，只不过2D CFAR的话忽略的是CUT紧邻周围的几圈点
% train也是CUT周围紧邻的几圈点
% 所以下面保证的是选中的CUT不是在边界，不然无法使用2D CFAR进行优化
% 只对图像中心的一部分"安全区域"进行CFAR处理

% rowStart, rowEnd, colStart, colEnd
% 以上四个变量存放的是一个矩阵的四个角标
% 这样画出来的矩阵能够保证矩阵中任意的点被选择为CUT，都能够进行2D-CFAR算法
[nRow, nCol] = size(value);
colStart = cfarParam.train(2) + cfarParam.guard(2) + 1;
colEnd =  nCol - (cfarParam.train(2) + cfarParam.guard(2));
rowStart = cfarParam.train(1) + cfarParam.guard(1) + 1;
rowEnd = nRow - (cfarParam.train(1) + cfarParam.guard(1));

% iCut 就是存放上面矩阵中所有的坐标对，本质是2 × N 的矩阵
iCut = [repmat(rowStart : rowEnd, 1, colEnd - colStart + 1); repelem(colStart : colEnd, rowEnd - rowStart + 1)];

% 建立2D CFAR模型
cfar = phased.CFARDetector2D('TrainingBandSize', cfarParam.train, 'GuardBandSize', ...
    cfarParam.guard, 'ProbabilityFalseAlarm', cfarParam.pfa, 'ThresholdOutputPort', true);
% 执行CFAR, 获得检测阈值
[~, th] = cfar(value, iCut);
% 将阈值向量转换为矩阵，这个矩阵就是 2D-CFAR 生成的噪声估计矩阵
th = reshape(th, [rowEnd - rowStart + 1, colEnd - colStart + 1]);
% 向裁剪后尺寸的阈值矩阵周围补正无穷, 使其恢复裁剪前的尺寸
th = [Inf * ones(nRow, colStart - 1), [Inf * ones(rowStart - 1, colEnd - colStart + 1); ...
    th; Inf * ones(nRow - rowEnd, colEnd - colStart + 1)], Inf * ones(nRow, nCol - colEnd)];
% 加入额外阈值
th = th + cfarParam.extraTh;
% 获得检测结果
[ptIdxCol, ptIdxRow] = find(value' - th' > 0); % 注意转置, 以使点云按照行-列的优先级排序
