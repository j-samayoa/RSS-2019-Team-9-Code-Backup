from math import *
import scipy.stats

def prob(zt,zts):

    alpha_hit = 0.74
    alpha_short = 0.07
    alpha_max = 0.07
    alpha_rand = 0.12
    sigma = 0.5
    zmax = 10.0
    #zts = 7.0
    zt = float(zt)
    zts = float(zts)

    if 0.0 <= zt <= zmax:
        #p_hit = 1/sqrt(2*pi*sigma**2)*exp(-((zt-zts)**2)/(2*sigma**2))
        p_hit = scipy.stats.norm(zts,sigma).pdf(zt)
    else:
        p_hit = 0.0
    
    #print(p_hit)

    if 0.0 <= zt <= zts:
        #print("HIIIIIIIIIIIII")
        p_short = (2.0/zts)*(1.0-zt/zts)
        #print(p_short)
    else:
        p_short = 0.0
            
    #print(p_short)

    #print("zt: ",zt)
    #print("zmax: ",zmax)
    if zt >= zmax:
        #print("Greater")
        p_max = 1.0
    else:
        p_max = 0.0

    #print(p_max)

    if 0.0 <= zt < zmax:
        p_rand = 1.0/zmax
    else:
        p_rand = 0.0

    #print(p_rand)

    p = alpha_hit*p_hit + alpha_short*p_short + alpha_max * p_max + alpha_rand*p_rand

    return p
