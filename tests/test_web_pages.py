"""Content of the static pages in docs/: music and credits, the user guide, the
glossary, the useful definitions, the landing page, and the links between them."""
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demos.catalog import CATALOG  # noqa: E402

DOCS = ROOT / "docs"
AUDIO = ["gymnopedie1.mp3", "gnossienne1.mp3", "gnossienne3.mp3", "meditation-impromptu-01.mp3",
         "meditation-impromptu-02.mp3", "clean-soul.mp3", "light-thought-1.mp3"]


def page(name):
    """The page without its inline style sheets and scripts."""
    html = (DOCS / name).read_text(encoding="utf-8")
    return re.sub(r'<(style|script) data-inline="[^"]+">.*?</\1>', "", html, flags=re.S)


class StaticPagesTest(unittest.TestCase):

    def test_music_files_and_credits_are_bundled(self):
        for name in AUDIO + ["CREDITS.md"]:
            self.assertTrue((DOCS / "audio" / name).is_file(), name)
        credits = (DOCS / "audio" / "CREDITS.md").read_text(encoding="utf-8")
        for name in AUDIO:
            self.assertIn(name, credits)
        music = (DOCS / "js" / "lib" / "music.js").read_text(encoding="utf-8")
        for name in AUDIO:
            self.assertIn(f"audio/{name}", music)

    def test_the_player_holds_the_pages(self):
        html = page("player.html")
        self.assertIn('<iframe id="app"', html)
        self.assertIn('data-inline="js/player.js"', (DOCS / "player.html").read_text(encoding="utf-8"))

    def test_user_guide(self):
        html = page("guide.html")
        for section in ("The landing page", "A demo page", "Ways to interact"):
            self.assertIn(section, html)

    def test_glossary_links_open_existing_demos(self):
        html = page("glossary.html")
        for term in ("Deformation Jacobian", "Broyden update", "Loss of observability"):
            self.assertIn(term, html)
        linked = set(re.findall(r'href="demo\.html\?id=([\w-]+)"', html))
        self.assertTrue(linked)
        self.assertLessEqual(linked, set(CATALOG))

    def test_useful_definitions(self):
        html = page("definitions.html")
        for term in ("Null space", "Range", "Pseudoinverse", "Quasistatic manipulation"):
            self.assertIn(term, html)
        anchors = set(re.findall(r'href="#([\w-]+)"', html))
        self.assertTrue(anchors)
        self.assertLessEqual(anchors, set(re.findall(r'id="([\w-]+)"', html)))   # no broken index links
        linked = set(re.findall(r'href="demo\.html\?id=([\w-]+)"', html))
        self.assertTrue(linked)
        self.assertLessEqual(linked, set(CATALOG))

    def test_landing_actions_and_palette_button(self):
        html = page("index.html")
        for action in ('href="guide.html"', 'href="glossary.html"', 'href="definitions.html"', 'id="citeButton"'):
            self.assertIn(action, html)
        for name in ("index.html", "demo.html", "guide.html", "glossary.html", "definitions.html"):
            self.assertIn('id="paletteButton"', page(name), name)

    def test_every_demo_has_a_lesson_a_page_module_and_a_session(self):
        sim = (DOCS / "js" / "sim" / "catalog.js").read_text(encoding="utf-8")
        registry = (DOCS / "js" / "demos" / "index.js").read_text(encoding="utf-8")
        for demo_id in CATALOG:
            self.assertTrue((DOCS / "lessons" / f"{demo_id}.html").is_file(), demo_id)
            self.assertTrue((DOCS / "js" / "demos" / f"{demo_id}.js").is_file(), demo_id)
            self.assertIn(f"entry('{demo_id}'", sim, demo_id)
            self.assertRegex(registry, rf"\b{demo_id}\b")


if __name__ == "__main__":
    unittest.main()
