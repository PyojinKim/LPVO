%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:   Yi Zhou                                                          *
% Contact:  yi.zhou@anu.com                                                  *
% License:  Copyright (c) 2016 Yi Zhou, ANU. All rights reserved.            *
%                                                                            *
% Redistribution and use the code, with or without                           *
% modification, are permitted provided that the following conditions         *
% are met:                                                                   *
% * Redistributions of source code must retain the above copyright           *
%   notice, this list of conditions and the following disclaimer.            *
% * Neither the name of ANU nor the names of its contributors may be         *
%   used to endorse or promote products derived from this software without   *
%   specific prior written permission.                                       *
%                                                                            *
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
% ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
% FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
% SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
% OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
% SUCH DAMAGE.                                                               *
%*****************************************************************************/

function [MF_nonRd,bin,succ_rate] = ClusterMMF(MF_can,ratio_th)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function aims at removing the redundancy of the MF (i.e. to achieve
% unique canonical) by doing non-maximum suppression.
% output:
%       MF_nonRd: non redundant MF rotation matrices.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Here we utilize a more careful and more smart strategy to find the dominant bins
MF_nonRd = [];
bin = [];
succ_rate = [];
histStart = 0;histStep = 0.1;histEnd = 2;
HasPeak = 1;
numMF_can = numel(MF_can);
numMF = numMF_can;
rng(0,'twister');
numMF_nonRd = 0;


while (HasPeak == 1)
    % randomly pick a R from updated MF_can
    R = MF_can{randi([1 numMF],1,1)};
    tempAA = vrrotmat2vec(R.' * R);
    PaaPick = tempAA(4)*[tempAA(1),tempAA(2),tempAA(3)]';
    Paa = zeros(3,numMF);
    d = zeros(numMF,1);
    for i = 1:numMF
        Rp = R.' * MF_can{i};
        tempAA = vrrotmat2vec(Rp);
        Paa(:,i) = tempAA(4)*[tempAA(1),tempAA(2),tempAA(3)]';
        d(i) = norm(PaaPick-Paa(:,i));
    end
    bin = EasyHist(d,histStart,histStep,histEnd);
    
    
    % check if there are peaks
    % Let's try something smart, if the ratio_th is
    HasPeak = 0;
    for k = 1:numel(bin)
        if (numel(bin{k})/numMF_can > ratio_th)
            HasPeak = 1;
            break;
        end
    end
    if (HasPeak == 0)
        return;
    end
    
    
    % check whether the dominant bin happens at zero
    if (numel(bin{1})/numMF_can > ratio_th)
        
        % calculate the mean, insert the R into the MF_nonRd
        MeanPaaTemp = mean(Paa(:,bin{1}),2);
        aa = MeanPaaTemp'/norm(MeanPaaTemp);
        RTemp = vrrotvec2mat([aa,norm(MeanPaaTemp)]);
        
        
        % compensate the rotation matrix
        if (isnan(sum(RTemp(:))) && (norm(MeanPaaTemp) == 0))
            numMF_nonRd = numMF_nonRd + 1;
            MF_nonRd{numMF_nonRd} = R;
            succ_rate{numMF_nonRd} = numel(bin{1})/numMF_can;
        else
            numMF_nonRd = numMF_nonRd + 1;
            MF_nonRd{numMF_nonRd} = R * RTemp;
            succ_rate{numMF_nonRd} = numel(bin{1})/numMF_can;
        end
        
        
        % update MF_can, clean the ones in the found mode
        for j = 1:numel(bin{1})
            MF_can{bin{1}(j)} = [];
        end
        MF_can = MF_can(~cellfun('isempty',MF_can));
        numMF = numel(MF_can);
        
        
        % one dominant MF
        if (numMF == 0)
            return;
        end
    else
        continue;
    end
end

end


function bin = EasyHist(data,first,step,last)
numData = numel(data);
numBin = (last - first) / step;
bin = cell(numBin,1);
for i = 1:numBin
    down = (i-1)*step + first;
    up = down + step;
    for j = 1:numData
        if (data(j) >= down && data(j) < up)
            bin{i} = [bin{i},j];
        end
    end
end
end