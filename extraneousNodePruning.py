pruning(path):
	n = sizeof(path)
	i=1
	removed = 0
	while(i<n-1-removed):
		if(collides(path[i-1],path[i+1])==True):
			path.pop(i)
			removed+=1
		else:
			i+=1