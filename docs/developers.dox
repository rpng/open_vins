/**


@page developers Developers guide


@section developers-addpage Adding a website page

1.  Add a `doc/pagename.dox` file, copy up-to-date license header and add your
    name + year to it, if not already there
2.  If the page is top-level, list it in `doc/00-page-order.dox` to ensure it
    gets listed at a proper place
3.  If the page is not top-level, list it using @c \@subpage in its parent page
4.  Add a @c \@brief documentation, if applicable
5.  Populate it, see @ref coding-style for more information



@section developers-notes Helpful commands


- More details on how to format your documentation can be found here:
    - http://www.doxygen.nl/manual/formulas.html
    - https://mcss.mosra.cz/css/components/#math
    - https://mcss.mosra.cz/css/components/
- Use the inline commands for latex `\f$ formula here \f$`
- Use block to have equation centered on page `\f[ biog formula \f]`



@section developers-building Building the documentation site

1. Clone the m.css repository which has scripts to build
2. https://github.com/mosra/m.css
3. You will need to install python3.6
    - `sudo apt-get install python3.6`
    - `sudo pip3.6 install jinja2 Pygments`
    - `sudo apt install texlive-base texlive-latex-extra texlive-fonts-extra texlive-fonts-recommended`
4. Go into the documentation folder and build
    - `cd m.css/documentation/`
    - `python3.6 doxgen.py <path_to_Doxyfile-mcss>`
5. This should then build the documentation website
6. Open the html page in the `doxgen_generated` folder








*/