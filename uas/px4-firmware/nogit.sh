find . | grep -Pe "\.git$" | while read file; do mv $file $file.hidden; done
