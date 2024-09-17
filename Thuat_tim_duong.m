% Kích thước của môi trường
gridSize = 100; % Kích thước lưới 100x100
map = zeros(gridSize, gridSize); % Khởi tạo môi trường không gian 2D (lưới)

% Xác định các chướng ngại vật (ở đây là hình chữ nhật)
obstacles = [
    20, 20, 40, 40;  % Chướng ngại vật 1: từ (20,20) đến (40,40)
    60, 60, 80, 80;  % Chướng ngại vật 2: từ (60,60) đến (80,80)
    10, 70, 30, 90   % Chướng ngại vật 3: từ (10,70) đến (30,90)
];

% Đánh dấu chướng ngại vật trong bản đồ (giá trị 1 sẽ là chướng ngại vật)
for i = 1:size(obstacles, 1)
    map(obstacles(i,2):obstacles(i,4), obstacles(i,1):obstacles(i,3)) = 1;
end

% Vẽ bản đồ với các chướng ngại vật
figure;
imagesc(map); % Hiển thị hình ảnh của bản đồ
colormap(gray); % Thiết lập màu xám cho hình ảnh (chướng ngại vật là màu đen)
hold on;

% Xác định điểm bắt đầu và kết thúc
start = [5, 5];   % Điểm bắt đầu
goal = [90, 90];  % Điểm đích

% Vẽ điểm bắt đầu và điểm kết thúc
plot(start(1), start(2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Điểm bắt đầu (màu xanh)
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);   % Điểm kết thúc (màu đỏ)

% Thông số cho RRT
max_iter = 5000;    % Số lần lặp tối đa
step_size = 5;      % Bước di chuyển mỗi lần mở rộng
tree = start;       % Khởi tạo cây với điểm bắt đầu

for i = 1:max_iter
    % Chọn ngẫu nhiên một điểm trong không gian
    rand_point = [randi([1, gridSize]), randi([1, gridSize])];
    
    % Tìm điểm gần nhất trong cây
    [nearest_node, idx] = min(sum((tree - rand_point).^2, 2));
    nearest_point = tree(idx, :);
    
    % Tính vector di chuyển
    direction = (rand_point - nearest_point) / norm(rand_point - nearest_point);
    new_point = nearest_point + step_size * direction;
    
    % Kiểm tra nếu điểm mới nằm trong bản đồ
    if new_point(1) < 1 || new_point(1) > gridSize || new_point(2) < 1 || new_point(2) > gridSize
        continue; % Bỏ qua nếu điểm mới ngoài biên
    end
    
    % Kiểm tra va chạm với chướng ngại vật
    if ~check_collision(new_point, map)
        tree = [tree; new_point]; % Mở rộng cây
        plot([nearest_point(1), new_point(1)], [nearest_point(2), new_point(2)], 'b', 'LineWidth', 1);
        pause(0.01); % Tạm dừng để hiển thị
    
        % Kiểm tra nếu đến gần điểm đích
        if norm(new_point - goal) < step_size
            plot([new_point(1), goal(1)], [new_point(2), goal(2)], 'g', 'LineWidth', 2);
            disp('Đã tìm thấy đường đi!');
            break;
        end
    end
end

hold off;

% Hàm kiểm tra va chạm
function collision = check_collision(point, map)
    % Chuyển đổi tọa độ thành số nguyên
    x = round(point(1));
    y = round(point(2));
    
    % Kiểm tra nếu tọa độ là chướng ngại vật
    if map(y, x) == 1
        collision = true;
    else
        collision = false;
    end
end
