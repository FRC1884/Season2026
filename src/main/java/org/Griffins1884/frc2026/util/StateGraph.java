package org.Griffins1884.frc2026.util;

public class StateGraph<V extends Enum<V>, T extends Transition<? extends Enum<V>>> {
  private final Object[][] adjacencyMap;
  private final Class<V> statesEnum;

  public StateGraph(Class<V> statesEnum) {
    int c = statesEnum.getEnumConstants().length;
    this.statesEnum = statesEnum;

    adjacencyMap = new Object[c][c];
  }

  private T getEdge(int s, int e) {
    return (T) adjacencyMap[s][e];
  }

  public T getEdge(V start, V end) {
    return getEdge(start.ordinal(), end.ordinal());
  }

  public void setEdge(T transition) {
    adjacencyMap[transition.getStartState().ordinal()][transition.getEndState().ordinal()] =
        transition;
  }

  public void addEdge(T transition) {
    setEdge(transition);
  }

  public void removeEdge(V start, V end) {
    adjacencyMap[start.ordinal()][end.ordinal()] = null;
  }
}
