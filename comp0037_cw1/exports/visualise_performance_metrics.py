import pandas as pd
import numpy as np

import textwrap

import matplotlib
import matplotlib.pyplot as plt

metrics = pd.read_csv('comp0037_cw1/exports/performanceMetrics.csv')

def plot_grouped_bar_graph(data, metric_col,  
                           title=None,y_label=None,
                           savefig=False, filename = None):
    # List of Planning algorithms
    planning_algorithms = data['PlanningAlgorithm'].unique()

    # Determine plot data and associated parameters
    label_name=data['mapName'].unique()
    n_maps=len(label_name)

    bar_data=[]
    for map_name in label_name:
        alg_data=[]
        for algorithm in planning_algorithms:
            alg_data.append(data[(data['PlanningAlgorithm']==algorithm) & (data['mapName']==map_name)][metric_col].iloc[0])
        bar_data.append(np.array(alg_data))

    # Define x tick locations
    x = np.arange(len(planning_algorithms)) 
    # Set width of the bar plot
    width = 0.8/n_maps 

    # Plot the bar graph
    fig, ax = plt.subplots(figsize=(20,10))
    if (n_maps%2==0):
        for i in range(n_maps//2):
            ax.bar(x - (2*i+1)*width/2, bar_data[n_maps//2 - (i+1)], width, label=label_name[n_maps//2 - (i+1)])
            ax.bar(x + (2*i+1)*width/2, bar_data[n_maps//2 + i], width, label=label_name[n_maps//2 + i])
    else:
        ax.bar(x, bar_data[n_maps//2], width,label=label_name[n_maps//2])
        for i in range(n_maps//2):
            ax.bar(x - (i+1)*width, bar_data[n_maps//2 - (i+1)], width,label=label_name[n_maps//2 - (i+1)])
            ax.bar(x + (i+1)*width, bar_data[n_maps//2 + i], width,label=label_name[n_maps//2 + i])
            
    # Check if title was passed
    if title is None:
        title = metric_col + ' for each algorithm'
    if y_label is None:
        y_label = metric_col

    # Add title, axis labels and tick labels
    ax.set_ylabel(y_label,fontsize=22,fontweight='bold')
    ax.set_xlabel('Algorithms',fontsize=22,fontweight='bold')
    ax.set_title(title,fontsize=24,fontweight='bold')
    ax.set_xticks(x)
    planning_algorithms=[textwrap.fill(name,22) for name in planning_algorithms]
    ax.set_xticklabels(planning_algorithms,fontsize=16)
    ax.legend(fontsize=18)

    # If savefig is True,  save the figure
    if savefig:
        if filename is None:
            filename = 'comp0037_cw1/report/images/'+title + '.png'
        plt.savefig(filename)

    fig.tight_layout()

for metric in metrics.columns[2:]:
    plot_grouped_bar_graph(metrics, metric, savefig=True)

for algorithm in metrics['PlanningAlgorithm'].unique():
    data=metrics[metrics['PlanningAlgorithm']==algorithm]
    fig, ax = plt.subplots(2,3,figsize=(36,18))
    for i in range(len(data.columns[2:])):
        if i<=2:
            row=0
            col=i
        else:
            row=1
            col=i-3
        ax[row][col].bar(data['mapName'],data[data.columns[i+2]])
        ax[row][col].set_title(data.columns[i+2]+' for each map',fontsize=22,fontweight='bold')
        ax[row][col].set_xlabel('Map type',fontsize=18,fontweight='bold')
        ax[row][col].set_ylabel(data.columns[i+2],fontsize=18,fontweight='bold')
        ax[row][col].tick_params(labelsize=16)
    ax[-1][-1].axis('off')
    plt.suptitle(algorithm,fontsize=25,fontweight='bold',y=0.95)
    alg_name = ''.join(c for c in '_'.join(algorithm.split()) if c.isalnum() or c=='_')
    plt.savefig('comp0037_cw1/report/images/metrics_plot_'+alg_name+'.png')
    