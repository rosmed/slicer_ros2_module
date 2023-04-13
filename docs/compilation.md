Required packages on Ubuntu 20.04:
```sh
sudo apt install latexmk texlive-latex-extra
pip install sphinx
pip install -r requirements.txt
```

Compiling the pdf version:
```sh
make latexpdf
evince _build/latex/slicerros2.pdf
```