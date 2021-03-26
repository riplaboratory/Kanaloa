`relabelScript.py` is based off of [this article](https://code.tutsplus.com/tutorials/quick-tip-how-to-make-changes-to-multiple-files-within-a-directory-using-python--cms-26452). The code goes into a directory on your system opens and read all text files and replaces the matching string to a new string. In this case, I modified the code to use regular expressions (regex) rather than using basic strings. 

In my case, this was used for a text file in YOLO format. The following is an example:
```
3 0.445703 0.301389 0.036719 0.058333
3 0.557031 0.362500 0.026562 0.041667
3 0.826562 0.281250 0.020313 0.029167
1 0.745313 0.306944 0.026562 0.100000
1 0.720313 0.271528 0.012500 0.054167
0 0.946094 0.288194 0.028125 0.073611
1 0.819922 0.250694 0.007031 0.026389
1 0.939063 0.256944 0.012500 0.038889
```
The following lines are from `relabelScript.py`
```python
    # (?<=) positive look behind
    # \n newline
    # 3 literal 3
    # '(?<=\n)3' looks behind any '3's for newline characters
    regex = re.compile('(?<=\n)3')
    read_file = regex.sub('2', read_file)
```
+ `regex` looks for the regular expression that will need be recompiled.
+ `read_file` will substitute the matching expressions and replace it with '2'

This is how the text file would look after
```
3 0.445703 0.301389 0.036719 0.058333
2 0.557031 0.362500 0.026562 0.041667
2 0.826562 0.281250 0.020313 0.029167
1 0.745313 0.306944 0.026562 0.100000
1 0.720313 0.271528 0.012500 0.054167
0 0.946094 0.288194 0.028125 0.073611
1 0.819922 0.250694 0.007031 0.026389
1 0.939063 0.256944 0.012500 0.038889
```
To replace the first '3' with '2', we need to change `regex` to read for the first string matching '3'. It is as simple as
```python
    # ^ start of string
    # 3 literal 3
    # ^3 literal 3 in start of string
    regex = re.compile('^3')
```
After running the script twice, all files in the directory should be updated. This is the final result
```
2 0.445703 0.301389 0.036719 0.058333
2 0.557031 0.362500 0.026562 0.041667
2 0.826562 0.281250 0.020313 0.029167
1 0.745313 0.306944 0.026562 0.100000
1 0.720313 0.271528 0.012500 0.054167
0 0.946094 0.288194 0.028125 0.073611
1 0.819922 0.250694 0.007031 0.026389
1 0.939063 0.256944 0.012500 0.038889
```

## Useful links
[Regex 101](https://regex101.com/)