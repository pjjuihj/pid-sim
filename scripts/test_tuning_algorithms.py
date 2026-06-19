"""tuning_algorithms.py 单元测试。"""

import sys
import os
import random

sys.path.insert(0, os.path.dirname(__file__))

from tuning_algorithms import PIDParams, TuningResult, GridSearch, GeneticAlgorithm


class TestPIDParams:
    """测试 PIDParams dataclass。"""

    def test_creation(self):
        p = PIDParams(kp=1.0, ki=2.0, kd=3.0)
        assert p.kp == 1.0
        assert p.ki == 2.0
        assert p.kd == 3.0

    def test_equality(self):
        p1 = PIDParams(kp=1.0, ki=2.0, kd=3.0)
        p2 = PIDParams(kp=1.0, ki=2.0, kd=3.0)
        assert p1 == p2

    def test_inequality(self):
        p1 = PIDParams(kp=1.0, ki=2.0, kd=3.0)
        p2 = PIDParams(kp=1.0, ki=2.0, kd=4.0)
        assert p1 != p2

    def test_zero_params(self):
        p = PIDParams(kp=0.0, ki=0.0, kd=0.0)
        assert p.kp == 0.0
        assert p.ki == 0.0
        assert p.kd == 0.0

    def test_negative_params(self):
        p = PIDParams(kp=-1.0, ki=-2.0, kd=-3.0)
        assert p.kp == -1.0
        assert p.ki == -2.0
        assert p.kd == -3.0


class TestTuningResult:
    """测试 TuningResult dataclass。"""

    def test_creation(self):
        p = PIDParams(kp=1.0, ki=2.0, kd=3.0)
        r = TuningResult(params=p, score=0.5, method='grid')
        assert r.params == p
        assert r.score == 0.5
        assert r.method == 'grid'

    def test_equality(self):
        p = PIDParams(kp=1.0, ki=2.0, kd=3.0)
        r1 = TuningResult(params=p, score=0.5, method='grid')
        r2 = TuningResult(params=p, score=0.5, method='grid')
        assert r1 == r2

    def test_different_scores(self):
        p = PIDParams(kp=1.0, ki=2.0, kd=3.0)
        r1 = TuningResult(params=p, score=0.5, method='grid')
        r2 = TuningResult(params=p, score=0.8, method='grid')
        assert r1 != r2

    def test_different_methods(self):
        p = PIDParams(kp=1.0, ki=2.0, kd=3.0)
        r1 = TuningResult(params=p, score=0.5, method='grid')
        r2 = TuningResult(params=p, score=0.5, method='ga')
        assert r1 != r2


class TestGridSearch:
    """测试 GridSearch。"""

    def test_init_defaults(self):
        gs = GridSearch()
        assert gs.kp_range == (10, 200)
        assert gs.ki_range == (0, 10)
        assert gs.kd_range == (0, 50)
        assert gs.steps == 10

    def test_init_custom(self):
        gs = GridSearch(kp_range=(0, 10), ki_range=(0, 5), kd_range=(0, 10), steps=5)
        assert gs.kp_range == (0, 10)
        assert gs.ki_range == (0, 5)
        assert gs.kd_range == (0, 10)
        assert gs.steps == 5

    def test_search_returns_sorted_results(self):
        gs = GridSearch(kp_range=(0, 10), ki_range=(0, 1), kd_range=(0, 1), steps=3)
        results = gs.search(lambda p: abs(p.kp - 5))
        assert len(results) == 27  # 3^3
        scores = [r.score for r in results]
        assert scores == sorted(scores)

    def test_search_result_type(self):
        gs = GridSearch(kp_range=(0, 5), ki_range=(0, 1), kd_range=(0, 1), steps=2)
        results = gs.search(lambda p: p.kp + p.ki + p.kd)
        assert all(isinstance(r, TuningResult) for r in results)

    def test_search_all_params_in_range(self):
        gs = GridSearch(kp_range=(10, 20), ki_range=(0, 5), kd_range=(0, 10), steps=5)
        results = gs.search(lambda p: 0)
        for r in results:
            assert 10 <= r.params.kp <= 20
            assert 0 <= r.params.ki <= 5
            assert 0 <= r.params.kd <= 10

    def test_search_finds_minimum(self):
        gs = GridSearch(kp_range=(0, 10), ki_range=(0, 1), kd_range=(0, 1), steps=11)
        results = gs.search(lambda p: abs(p.kp - 5))
        best = results[0]
        assert best.params.kp == 5.0

    def test_search_with_constant_function(self):
        gs = GridSearch(kp_range=(0, 5), ki_range=(0, 5), kd_range=(0, 5), steps=3)
        results = gs.search(lambda p: 42.0)
        assert len(results) == 27
        assert all(r.score == 42.0 for r in results)

    def test_search_with_negative_scorer(self):
        gs = GridSearch(kp_range=(0, 10), ki_range=(0, 1), kd_range=(0, 1), steps=5)
        results = gs.search(lambda p: -p.kp)
        best = results[0]  # lowest score = most negative
        assert best.params.kp == 10.0


class TestGeneticAlgorithm:
    """测试 GeneticAlgorithm。"""

    def test_init_defaults(self):
        ga = GeneticAlgorithm()
        assert ga.kp_range == (10, 200)
        assert ga.ki_range == (0, 10)
        assert ga.kd_range == (0, 50)
        assert ga.pop == 50
        assert ga.gens == 100
        assert ga.mut == 0.1

    def test_init_custom(self):
        ga = GeneticAlgorithm(kp_range=(0, 10), ki_range=(0, 5), kd_range=(0, 10),
                              pop=20, gens=50, mut=0.2)
        assert ga.kp_range == (0, 10)
        assert ga.ki_range == (0, 5)
        assert ga.kd_range == (0, 10)
        assert ga.pop == 20
        assert ga.gens == 50
        assert ga.mut == 0.2

    def test_optimize_returns_correct_number_of_generations(self):
        random.seed(42)
        ga = GeneticAlgorithm(pop=10, gens=5)
        results = ga.optimize(lambda p: p.kp + p.ki + p.kd)
        assert len(results) == 5

    def test_optimize_result_type(self):
        random.seed(42)
        ga = GeneticAlgorithm(pop=10, gens=5)
        results = ga.optimize(lambda p: p.kp)
        assert all(isinstance(r, TuningResult) for r in results)
        assert all(r.method == 'ga' for r in results)

    def test_optimize_params_in_range(self):
        random.seed(42)
        ga = GeneticAlgorithm(kp_range=(0, 100), ki_range=(0, 5), kd_range=(0, 10),
                              pop=20, gens=5)
        results = ga.optimize(lambda p: abs(p.kp - 50))
        for r in results:
            assert 0 <= r.params.kp <= 100
            assert 0 <= r.params.ki <= 5
            assert 0 <= r.params.kd <= 10

    def test_optimize_improves_over_generations(self):
        random.seed(42)
        ga = GeneticAlgorithm(kp_range=(0, 100), ki_range=(0, 5), kd_range=(0, 10),
                              pop=30, gens=20)
        results = ga.optimize(lambda p: abs(p.kp - 50) + abs(p.ki - 2.5) + abs(p.kd - 5))
        # Last generation should generally be better than first
        first_score = results[0].score
        last_score = results[-1].score
        # With random seed, improvement is expected
        assert last_score <= first_score

    def test_optimize_smallest_population(self):
        random.seed(42)
        ga = GeneticAlgorithm(pop=4, gens=3)
        results = ga.optimize(lambda p: p.kp)
        assert len(results) == 3

    def test_optimize_with_constant_function(self):
        random.seed(42)
        ga = GeneticAlgorithm(pop=10, gens=5)
        results = ga.optimize(lambda p: 10.0)
        assert len(results) == 5
        # All should have score 10.0
        for r in results:
            assert r.score == 10.0
