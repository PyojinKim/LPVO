function [m_j] = projectManhattanFrame(R_Mc, unitVector, halfApexAngle)

% convert to Manhattan frame
n_j = R_Mc * unitVector;


% find inside cone
lambda = sqrt(n_j(1,:).*n_j(1,:) + n_j(2,:).*n_j(2,:));
index = find(lambda < sin(halfApexAngle));
n_j_inlier = n_j(:,index);
tan_alfa = lambda(index)./abs(n_j(3,index));
alfa = asin(lambda(index));
m_j = [alfa./tan_alfa.*n_j_inlier(1,:)./n_j_inlier(3,:);
    alfa./tan_alfa.*n_j_inlier(2,:)./n_j_inlier(3,:)];


% valid projection result
if (isempty(m_j))
    m_j = [];
    return;
end
select = ~isnan(m_j);
select2 = select(1,:).*select(2,:);
select3 = find(select2 == 1);
m_j = m_j(:,select3);


end

