TEX = pdflatex -interaction batchmode -file-line-error
BIB = biber --quiet

REPORT = report
BIBFILE = $(REPORT)
ARTIFACTS := *.aux *.log *.bbl *.bcf *.blg *.out *.lot *.lof *.xml *.toc *.gz

all: clean-all $(REPORT).pdf

.PHONY: clean clean-all
clean:
	rm -fv $(ARTIFACTS)
	cd sections && rm -fv *.aux

clean-all: clean
	rm -f $(REPORT).pdf

$(REPORT).pdf: $(REPORT).tex
	$(TEX) $(REPORT)
	$(BIB) $(BIBFILE)
	$(TEX) $(REPORT)
