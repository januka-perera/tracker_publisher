import matplotlib.pyplot as plt

def prettify_plot(filename):
    """
    Prettifies the current Matplotlib plot for inclusion in a LaTeX report.
    """
    # Set font family to match LaTeX font
    plt.rcParams['font.family'] = 'serif'

    # Increase font size of tick labels
    plt.xticks(fontsize=10)
    plt.yticks(fontsize=10)

    # Remove top and right spines
    plt.gca().spines['top'].set_visible(False)
    plt.gca().spines['right'].set_visible(False)

    # Set linewidth of remaining spines
    plt.gca().spines['left'].set_linewidth(0.15)
    plt.gca().spines['bottom'].set_linewidth(0.15)

    # Adjust subplot padding
    plt.subplots_adjust(left=0.15, bottom=0.15)

    # Set background color
    plt.gca().set_facecolor('white')

    # Retrieve existing labels, title, and legend from the plot
    xlabel = plt.gca().get_xlabel()
    ylabel = plt.gca().get_ylabel()
    title = plt.gca().get_title()
    # legend = plt.gca().get_legend()

    # Add additional customizations based on existing labels, title, and legend
    if xlabel:
        plt.xlabel(xlabel, fontsize=12, fontfamily='serif')
    if ylabel:
        plt.ylabel(ylabel, fontsize=12, fontfamily='serif')
    if title:
        plt.title(title, fontsize=14, fontfamily='serif')
    # if legend:
    #     for text in legend.get_texts():
    #         text.set_fontfamily('serif')
    #     plt.legend(fontsize=10)

    # Add additional customizations as needed for your specific plot
    # For example, setting axis labels, title, legends, etc.

    # Save the plot as EPS for LaTeX compatibility
    plt.savefig(filename+'.png', dpi=1000, bbox_inches='tight', format = 'png')

    # Show the prettified plot (optional)
    # plt.show()