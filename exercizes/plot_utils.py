# -*- coding: utf-8 -*-
"""
Created on Fri Jan 16 09:16:56 2015

@author: adelpret
"""
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

DEFAULT_FONT_SIZE = 20;
DEFAULT_AXIS_FONT_SIZE = DEFAULT_FONT_SIZE;
DEFAULT_LINE_WIDTH = 4; #13;
DEFAULT_MARKER_SIZE = 6;
DEFAULT_FONT_FAMILY = 'sans-serif'
DEFAULT_FONT_SIZE = DEFAULT_FONT_SIZE;
DEFAULT_FONT_SERIF = ['Times New Roman', 'Times','Bitstream Vera Serif', 'DejaVu Serif', 'New Century Schoolbook', 'Century Schoolbook L', 'Utopia', 'ITC Bookman', 'Bookman', 'Nimbus Roman No9 L', 'Palatino', 'Charter', 'serif'];
DEFAULT_FIGURE_FACE_COLOR = 'white'    # figure facecolor; 0.75 is scalar gray
DEFAULT_LEGEND_FONT_SIZE = DEFAULT_FONT_SIZE;
DEFAULT_AXES_LABEL_SIZE = DEFAULT_FONT_SIZE;  # fontsize of the x any y labels
DEFAULT_TEXT_USE_TEX = False;
LINE_ALPHA = 0.9;
SAVE_FIGURES = False;
FILE_EXTENSIONS = ['png']; #'pdf']; #['png']; #,'eps'];
FIGURES_DPI = 150;
SHOW_LEGENDS = False;
LEGEND_ALPHA = 0.5;
SHOW_FIGURES = False;
FIGURE_PATH = './';
LINE_WIDTH_RED = 0; # reduction of line width when plotting multiple lines on same plot
LINE_WIDTH_MIN = 1;
BOUNDS_COLOR = 'silver';

#legend.framealpha    : 1.0    # opacity of of legend frame
#axes.hold           : True    # whether to clear the axes by default on
#axes.linewidth      : 1.0     # edge linewidth
#axes.titlesize      : large   # fontsize of the axes title
#axes.color_cycle    : b, g, r, c, m, y, k  # color cycle for plot lines
#xtick.labelsize      : medium # fontsize of the tick labels
#figure.dpi       : 80      # figure dots per inch
#image.cmap   : jet               # gray | jet etc...
#savefig.dpi         : 100      # figure dots per inch
#savefig.facecolor   : white    # figure facecolor when saving
#savefig.edgecolor   : white    # figure edgecolor when saving
#savefig.format      : png      # png, ps, pdf, svg
#savefig.jpeg_quality: 95       # when a jpeg is saved, the default quality parameter.
#savefig.directory   : ~        # default directory in savefig dialog box,
                                # leave empty to always use current working directory

def create_empty_figure(nRows=1, nCols=1, figsize=(7, 7), spinesPos=None,sharex=True):
    f, ax = plt.subplots(nRows,nCols, figsize=figsize, sharex=sharex);
    mngr = plt.get_current_fig_manager()
#    mngr.window.setGeometry(50,50,1080,720);

    if(spinesPos!=None):
        if(nRows*nCols>1):
            for axis in ax.reshape(nRows*nCols):
                movePlotSpines(axis, spinesPos);
        else:
            movePlotSpines(ax, spinesPos);
    return (f, ax);
    
def movePlotSpines(ax, spinesPos):
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')
    ax.xaxis.set_ticks_position('bottom')
    ax.spines['bottom'].set_position(('data',spinesPos[0]))
    ax.yaxis.set_ticks_position('left')
    ax.spines['left'].set_position(('data',spinesPos[1]))
    
def setAxisFontSize(ax, size):
    for label in ax.get_xticklabels() + ax.get_yticklabels():
        label.set_fontsize(size)
        label.set_bbox(dict(facecolor='white', edgecolor='None', alpha=0.65))
        
mpl.rcdefaults()
mpl.rcParams['lines.linewidth']     = DEFAULT_LINE_WIDTH;
mpl.rcParams['lines.markersize']    = DEFAULT_MARKER_SIZE;
mpl.rcParams['patch.linewidth']     = 1;
mpl.rcParams['font.family']         = DEFAULT_FONT_FAMILY;
mpl.rcParams['font.size']           = DEFAULT_FONT_SIZE;
mpl.rcParams['font.serif']          = DEFAULT_FONT_SERIF;
mpl.rcParams['text.usetex']         = DEFAULT_TEXT_USE_TEX;
mpl.rcParams['axes.labelsize']      = DEFAULT_AXES_LABEL_SIZE;
mpl.rcParams['legend.fontsize']     = DEFAULT_LEGEND_FONT_SIZE;
mpl.rcParams['figure.facecolor']    = DEFAULT_FIGURE_FACE_COLOR;
mpl.rcParams['figure.figsize']      = 23, 12 #12, 9 #

def plot3dQuantity(quantity, title, ax=None, boundUp=None, boundLow=None, yscale='linear', linestyle='k'):
    return plotNdQuantity(3, 1, quantity, title, ax, boundUp, boundLow, yscale, linestyle);
    
def plotNdQuantity(nRows, nCols, quantity, title="", ax=None, boundUp=None, boundLow=None, yscale='linear', 
                   linestyle='k--', sharey=False, margins=None):
    t = quantity.shape[0];
    n = quantity.shape[1];
    if(margins!=None):
        if(type(margins) is list):
            margins = [margins[0].reshape(t,1,n), margins[1].reshape(t,1,n)];
        else:
            margins = margins.reshape(t,1,n);
    return plotNdQuantityPerSolver(nRows, nCols, quantity.reshape(t,1,n), title, None, [linestyle], ax, 
                                   boundUp, boundLow, yscale, None, None, sharey, margins);
    
def plotNdQuantityPerSolver(nRows, nCols, quantity, title, solver_names, line_styles, ax=None, boundUp=None, boundLow=None, 
                            yscale='linear', subplot_titles=None, ylabels=None, sharey=False, margins=None, x=None):
    if(ax==None):
        f, ax = plt.subplots(nRows, nCols, sharex=True, sharey=sharey);
    ax = ax.reshape(nRows, nCols);
    k = 0;
    if(x==None):
        x = range(quantity.shape[0]);
    for j in range(nCols):
        for i in range(nRows):
            if(k<quantity.shape[2]):
                if(subplot_titles!=None):
                    ax[i,j].set_title(subplot_titles[k]);
                elif(i==0):
                    ax[i,j].set_title(str(k));  # set titles on first row only
                if(ylabels!=None):
                    ax[i,j].set_ylabel(ylabels[k]);
                    
                ymin = np.min(quantity[:,:,k]);
                ymax = np.max(quantity[:,:,k]);
                if(boundUp!=None):
                    if(len(boundUp.shape)==1):  # constant bound
                        if(boundUp[k]<2*ymax):
                            ymax = np.max([ymax,boundUp[k]]);
                        ax[i,j].plot([0, quantity.shape[0]-1], [boundUp[k], boundUp[k]], '--', color=BOUNDS_COLOR, alpha=LINE_ALPHA);
                    elif(len(boundUp.shape)==2):    # bound variable in time but constant for each solver
                        if(np.max(boundUp[:,k])<2*ymax):
                            ymax = np.max(np.concatenate(([ymax],boundUp[:,k])));
                        ax[i,j].plot(boundUp[:,k], '--', color=BOUNDS_COLOR, label='Upper bound', alpha=LINE_ALPHA);
                if(boundLow!=None):
                    if(len(boundLow.shape)==1):
                        if(boundLow[k]>2*ymin):
                            ymin = np.min([ymin,boundLow[k]]);
                        ax[i,j].plot([0, quantity.shape[0]-1], [boundLow[k], boundLow[k]], '--', color=BOUNDS_COLOR, alpha=LINE_ALPHA);
                    else:
                        if(np.min(boundLow[:,k])>2*ymin):
                            ymin = np.min(np.concatenate(([ymin],boundLow[:,k])));
                        ax[i,j].plot(boundLow[:,k], '--', color=BOUNDS_COLOR, label='Lower bound', alpha=LINE_ALPHA);
                lw = DEFAULT_LINE_WIDTH;
                for s in range(quantity.shape[1]):
                    p, = ax[i,j].plot(x, quantity[:,s,k], line_styles[s], alpha=LINE_ALPHA, linewidth=lw);
                    if(margins!=None):
                        if(type(margins) is list):
                            mp = margins[0];
                            mn = margins[1];
                        else:
                            mp = margins;
                            mn = margins;
                        ymax = np.max(np.concatenate(([ymax],quantity[:,s,k]+mp[:,s,k])));
                        ymin = np.min(np.concatenate(([ymin],quantity[:,s,k]-mn[:,s,k])));
                        ax[i,j].fill_between(x, quantity[:,s,k]+mp[:,s,k], quantity[:,s,k]-mn[:,s,k], alpha=0.15, linewidth=0, facecolor='green');
                    if(solver_names!=None):
                        p.set_label(solver_names[s]);
                    lw=max(LINE_WIDTH_MIN,lw-LINE_WIDTH_RED);
                ax[i,j].set_yscale(yscale);
                ax[i,j].xaxis.set_ticks(np.arange(0, x[-1], x[-1]/2));
                ax[i,j].yaxis.set_ticks([ymin, ymax]);
                if(ymax-ymin>5.0):
                    ax[i,j].yaxis.set_major_formatter(ticker.FormatStrFormatter('%0.0f'));
                elif(ymax-ymin>0.5):
                    ax[i,j].yaxis.set_major_formatter(ticker.FormatStrFormatter('%0.1f'));
                else:
                    ax[i,j].yaxis.set_major_formatter(ticker.FormatStrFormatter('%0.2f'));
                if(sharey==False):
                    ax[i,j].set_ylim([ymin-0.1*(ymax-ymin), ymax+0.1*(ymax-ymin)]);
                k += 1;
            else:
                ax[i,j].yaxis.set_major_formatter(ticker.FormatStrFormatter('%0.0f'));                
                
    if(SAVE_FIGURES):
        for ext in FILE_EXTENSIONS:
            plt.gcf().savefig(FIGURE_PATH+title.replace(' ', '_')+'.'+ext, format=ext, dpi=FIGURES_DPI, bbox_inches='tight');
    else:
        ax[nRows/2,0].set_ylabel(title);    
    if(SHOW_LEGENDS):
#        leg = ax[0,0].legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3, ncol=2, mode="expand", borderaxespad=0.)
        leg = ax[0,0].legend(loc='best');
#        leg.get_frame().set_alpha(LEGEND_ALPHA)
    return ax;
    
def plotQuantityPerSolver(quantity, title, solver_names, line_styles, yscale='linear', ylabel='', 
                          x=None, xlabel='', legend_location='best'):
    f, ax = plt.subplots();
    lw = DEFAULT_LINE_WIDTH;
    if(x==None):
        x = range(quantity.shape[0]);
        
    for i in range(len(solver_names)):
        ax.plot(x, quantity[:,i], line_styles[i], alpha=LINE_ALPHA, linewidth=lw);
        lw=max(lw-LINE_WIDTH_RED,LINE_WIDTH_MIN);
    ax.set_yscale(yscale);
    ax.set_ylabel(ylabel);
    ax.set_xlabel(xlabel);
    ymin = np.min(quantity);
    ymax = np.max(quantity);
    ax.set_ylim([ymin-0.1*(ymax-ymin), ymax+0.1*(ymax-ymin)]);
    if(SHOW_LEGENDS):
        leg = ax.legend(solver_names, loc=legend_location);
        leg.get_frame().set_alpha(LEGEND_ALPHA)
    if(SAVE_FIGURES):
        for ext in FILE_EXTENSIONS:
            plt.gcf().savefig(FIGURE_PATH+title.replace(' ', '_')+'.'+ext, format=ext, dpi=FIGURES_DPI, bbox_inches='tight');
    elif(ylabel==''):
        ax.set_ylabel(title);
        
        
def plotQuantityVsQuantityPerSolver(quantity, quantityPerSolver, legend, solver_names, line_styles, yscale='linear'):
    r=0;
    c=0;
    if(len(solver_names)==4 or len(solver_names)==3):
        r=2;
        c=2;
    elif(len(solver_names)==5 or len(solver_names)==6):
        r=2;
        c=3;
    else:
        print "ERROR in plotQuantityVsQuantityPerSolver, number of solvers not managed";
        return;
    f, ax = plt.subplots(r, c, sharex=True, sharey=True);
    for i in range(len(solver_names)):
        ax[i/c,i%c].plot(quantity[:,i], 'kx-', quantityPerSolver[:,i], line_styles[i], alpha=LINE_ALPHA);
        ax[i/c,i%c].set_ylabel(solver_names[i]);
        ax[i/c,i%c].set_yscale(yscale);
    if(SAVE_FIGURES):
        for ext in FILE_EXTENSIONS:
            f.savefig(FIGURE_PATH+(legend[0]+'_VS_'+legend[1]).replace(' ', '_')+'.'+ext, format=ext, dpi=FIGURES_DPI, bbox_inches='tight');
    if(SHOW_LEGENDS):
        leg = ax[0,0].legend(legend, loc='best');
        leg.get_frame().set_alpha(LEGEND_ALPHA)

def grayify_cmap(cmap):
    """Return a grayscale version of the colormap"""
    cmap = plt.cm.get_cmap(cmap)
    colors = cmap(np.arange(cmap.N))
    
    # convert RGBA to perceived greyscale luminance
    # cf. http://alienryderflex.com/hsp.html
    RGB_weight = [0.299, 0.587, 0.114]
    luminance = np.sqrt(np.dot(colors[:, :3] ** 2, RGB_weight))
    colors[:, :3] = luminance[:, np.newaxis]
    
    return cmap.from_list(cmap.name + "_grayscale", colors, cmap.N)
    
def saveFigure(title):
    if(SAVE_FIGURES):
        for ext in FILE_EXTENSIONS:
            plt.gcf().savefig(FIGURE_PATH+title.replace(' ', '_')+'.'+ext, format=ext, dpi=FIGURES_DPI, bbox_inches='tight');
