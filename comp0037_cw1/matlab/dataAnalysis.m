mydir = pwd;
idcs = strfind(mydir,filesep);
newdir = mydir(1:idcs(end)-1);

folder = filesep + "exports";
newdir = newdir + folder;


filename = filesep + "performanceMetrics.csv";
file_dir = newdir + filename;

% opts = detectImportOptions(file_dir);



M = readtable(file_dir);

columns = M.Properties.VariableNames(3:end);

planners = unique(M.PlanningAlgorithm);
alt_planners = ["A*:0", "A*:1", "A*:20", "A*:5", "A*:Euclidean", "A*:manhattan", "A*:Octile", "FIFO", "LIFO", "Dijkstra's", "Greedy"];

maps = unique(M.mapName);

% for map_num = 1:length(maps)
%     for planner_num = 1:length(planners)
%         
%     end
% end

data = zeros(length(maps), length(planners), length(columns));

% map = maps(1);
for map_num = 1:length(maps)
    for planner_num = 1:length(planners)
        for row_num = 1:length(M.PlanningAlgorithm)
            if strcmp(M.PlanningAlgorithm(row_num), planners(planner_num)) && strcmp(M.mapName(row_num),maps(map_num))
                correct_row = row_num;
                break;
            end
        end
        
        for col_num = 1:length(columns)
            data(map_num, planner_num, col_num) = M.(char(columns(col_num)))(correct_row);
        end
    end
end

for map_num = 1:length(maps)
    for col_num = 1:length(columns)
        data_to_plot = data(map_num, :, col_num);
        map_name = strrep(char(maps(map_num)),'_',' ');
        column_name =  char(columns(col_num));
        f = figure('visible','off');
        bar(data_to_plot);
        title(strcat(map_name, ':', column_name));
        set(gca,'xticklabel',alt_planners);
        xtickangle(45);
        saveas(gcf,strcat(char(maps(map_num)), '_', char(columns(col_num)),'.png'));
    end
end