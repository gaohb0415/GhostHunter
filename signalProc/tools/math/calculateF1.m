function [f1, precision, recall] = calculateF1(labels, predictions, positiveClass, negativeClass)
% 计算F1指标
% 加入了样本均衡
% 输入:
% 1. labels: 真实标签
% 2. predictions: 预测标签
% 3. positiveClass: 正类标签
% 4. positiveClass: 负类标签
% 输出:
% 1. f1
% 2. precision
% 3. recall
% 作者: 刘涵凯
% 更新: 2024-3-14

%% 混淆矩阵
TP = sum(predictions == positiveClass & labels == positiveClass);
FP = sum(predictions == positiveClass & labels == negativeClass);
FN = sum(predictions == negativeClass & labels == positiveClass);

%% 样本均衡
nP = sum(labels == positiveClass);
nN = sum(labels == negativeClass);
FP = FP / nN * nP;

%% 计算指标
precision = TP / (TP + FP);
recall = TP / (TP + FN);
f1 = 2 * (precision * recall) / (precision + recall);
