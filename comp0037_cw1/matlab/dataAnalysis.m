clear all

% Setting up directory variables for reading exported data
mydir = pwd;
idcs = strfind(mydir,filesep);
newdir = mydir(1:idcs(end)-1);

folder = filesep + "exports";
newdir = newdir + folder;

filename = filesep + "performanceMetrics.csv";
file_dir = newdir + filename;

% Reading data into a table
M = readtable(file_dir);

% Column Headings (Metric names)
columns = M.Properties.VariableNames(3:end);

% Planner names and alternate names for use in plot
planners = unique(M.PlanningAlgorithm);
alt_planners = ["A*:0", "A*:1", "A*:20", "A*:5", "A*:Euclidean", "A*:manhattan", "A*:Octile", "FIFO", "LIFO", "Dijkstra's", "Greedy"];

% Map names
maps = unique(M.mapName);

% Initializing empty data matrix (each metric for each map and planner)
data = zeros(length(maps), length(planners), length(columns));

% Iterating through the table to write data into matrix
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

% Plotting and saving each plot
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