from pathlib import Path

from modelica_routing_audit import audit, complementary, crosses_box, crossings


def test_crossings_allow_shared_endpoints_and_trunks():
    assert crossings([(0, 0), (10, 0)], [(5, -5), (5, 5)]) == [(5, 0)]
    assert not crossings([(0, 0), (10, 0)], [(5, 0), (5, 5)])
    assert not crossings([(0, 0), (10, 0)], [(0, 0), (5, 0), (5, 5)])


def test_scopes_visibility_and_non_code():
    text = '''model Outer
      // connect(a,b) annotation(Line(points={{0,0},{10,0}}));
      String description="connect(a,b)";
      connect(a,b) annotation(Line(points={{0,0},{10,0}}));
      connect(c,d) annotation(Line(points={{5,-5},{5,5}},visible=enabled));
      connect(e,f) annotation(Line(points={{7,-5},{7,5}},visible=false));
      model Inner
        connect(g,h) annotation(Line(points={{8,-5},{8,5}}));
      end Inner;
    end Outer;'''
    result = audit(text)
    assert result['routes'] == 3
    assert len(result['crossings']) == 1
    assert result['crossings'][0]['conditional']
    assert result['unresolved'][0]['reason'] == 'evaluate visibility per variant'
    assert result['visual_acceptance'] == 'not established'


def test_unknown_geometry_is_not_silently_accepted():
    result = audit('model A connect(a,b) annotation(Line(points=p)); end A;')
    assert result['unresolved']
    assert result['review_score'] is None


def test_straight_route_scores_above_unnecessary_jog():
    direct = audit('model A connect(a,b) annotation(Line(points={{0,0},{10,0}})); end A;')
    jog = audit('model A connect(a,b) annotation(Line(points={{0,0},{0,5},{10,5},{10,0}})); end A;')
    assert direct['review_score'] > jog['review_score']
    assert jog['extra_bends'] == 2


def test_symbol_crossing_and_overlap_without_flagging_connected_symbol():
    text = '''model A
      X source annotation(Placement(transformation(origin={0,0},extent={{-1,-1},{1,1}})));
      X obstacle annotation(Placement(transformation(origin={5,0},extent={{-1,-1},{1,1}})));
      X overlapping annotation(Placement(transformation(origin={5,0},extent={{-1,-1},{1,1}})));
      connect(source.y,outside) annotation(Line(points={{0,0},{10,0}}));
    end A;'''
    result = audit(text)
    assert {r['symbol'] for r in result['symbol_crossings']} == {'obstacle', 'overlapping'}
    assert len(result['symbol_overlaps']) == 1
    assert not crosses_box([(0, 1), (10, 1)], (4, -1, 6, 1))


def test_reversed_duplicate_connection_is_reported():
    text = '''model A
      connect(a,b) annotation(Line(points={{0,0},{10,0}}));
      connect(b,a) annotation(Line(points={{10,0},{0,0}}));
    end A;'''
    assert len(audit(text)['duplicate_connections']) == 1


def test_only_proven_exclusive_variants_may_share_a_placement():
    assert complementary('a and b', 'not (a and b)')
    assert not complementary('a', 'not a and b')
    assert not complementary('a or b', 'not a or b')
    text = '''model A
      X a annotation(Placement(visible=spatial,transformation(extent={{-1,-1},{1,1}})));
      X b annotation(Placement(visible=not spatial,transformation(extent={{-1,-1},{1,1}})));
    end A;'''
    result = audit(text)
    assert not result['symbol_overlaps']
    assert len(result['exclusive_overlays']) == 1


def test_all_local_symbols_have_nonoverlapping_placements():
    root = Path(__file__).resolve().parents[1]
    for folder in [root/'BobLib', root/'Tests/BobLibTest']:
        for path in folder.rglob('*.mo'):
            result = audit(path.read_text())
            assert not result['symbol_overlaps'], str(path)
            assert not result['duplicate_connections'], str(path)
