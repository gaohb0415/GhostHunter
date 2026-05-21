function peakSet = findTopLocalPeaks2D(roi_map, rangeAxis, angleAxis, nTop, minSepR, minSepA)
tmpMap = roi_map;
peakSet = zeros(0, 3);   % [peakR, peakA, peakVal]

for iPeak = 1 : nTop
    [peakVal, linIdx] = max(tmpMap(:));
    if peakVal <= 0
        break;
    end

    [rIdx, aIdx] = ind2sub(size(tmpMap), linIdx);
    peakR = rangeAxis(rIdx);
    peakA = angleAxis(aIdx);

    peakSet = [peakSet; peakR, peakA, peakVal];

    maskR = abs(rangeAxis - peakR) <= minSepR;
    maskA = abs(angleAxis - peakA) <= minSepA;
    tmpMap(maskR, maskA) = 0;
end
end