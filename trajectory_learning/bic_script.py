from sklearn.mixture import GaussianMixture as GMM
import numpy as np
import matplotlib.pyplot as plt

def SelBest(arr:list, X:int)->list:
    '''
    returns the set of X configurations with shorter distance
    '''
    dx=np.argsort(arr)[:X]
    return arr[dx]

data=np.loadtxt('BIC/combined.csv')

n_clusters=np.arange(2, 20)
bics=[]
bics_err=[]
iterations=20
for n in n_clusters:
    tmp_bic=[]
    for i in range(iterations):
        gmm=GMM(n, n_init=2).fit(data) 
        
        tmp_bic.append(gmm.bic(data))
        print(i)
    val=np.mean(SelBest(np.array(tmp_bic), int(iterations/5)))
    err=np.std(tmp_bic)
    bics.append(val)
    bics_err.append(err)

plt.figure(1)
plt.plot(n_clusters,bics, label='BIC')
plt.title("BIC Scores", fontsize=20)
plt.xticks(n_clusters)
plt.xlabel("Number of clusters")
plt.ylabel("Score")
plt.legend()

plt.figure(2)
plt.plot(n_clusters, np.gradient(bics), label='BIC')
plt.title("Gradient of BIC Scores", fontsize=20)
plt.xticks(n_clusters)
plt.xlabel("Number of clusters")
plt.ylabel("grad(BIC)")
plt.legend()

plt.show()
