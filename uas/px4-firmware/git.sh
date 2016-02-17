find . | grep -Pe "\.git\.hidden$" | while read file; do mv $file ${file:0:${#file}-7}; done
