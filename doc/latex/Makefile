IMG := $(wildcard img/*.jpg) $(wildcard img/*.png)
IMG := $(IMG:.jpg=.pdf)
IMG := $(IMG:.png=.pdf)

%.pdf: %.png
	convert $< $@

%.pdf: %.jpg
	convert $< $@

main.pdf: main.tex $(shell find . -name '*.tex' -or -name '*.bib') $(IMG)
	texi2pdf $<
	bibtex main
	texi2pdf $<

view: main.pdf
	xpdf $<

clean:
	${RM} main.aux main.idx main.log main.out main.pdf main.toc main.bbl main.blg main.lof
