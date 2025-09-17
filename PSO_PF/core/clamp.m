function data = clamp(data, limit, d)
% �ȼ���ԭ edgeLimit���� data ��ÿһ�вü��� limit ��������
% limit: d��2��ÿ�� [min max]
N = size(data,1);
for i = 1:d
    for j = 1:N
        if data(j,i) > limit(i,2)
            data(j,i) = limit(i,2);
        end
        if data(j,i) < limit(i,1)
            data(j,i) = limit(i,1);
        end
    end
end
end
