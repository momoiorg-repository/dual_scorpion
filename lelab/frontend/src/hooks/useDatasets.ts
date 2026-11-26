import { useCallback, useEffect, useState } from "react";
import { useApi } from "@/contexts/ApiContext";
import { DatasetItem, listDatasets } from "@/lib/replayApi";

export const useDatasets = () => {
  const { baseUrl, fetchWithHeaders } = useApi();
  const [datasets, setDatasets] = useState<DatasetItem[]>([]);
  const [loading, setLoading] = useState(true);

  const refresh = useCallback(() => {
    setLoading(true);
    listDatasets(baseUrl, fetchWithHeaders)
      .then(setDatasets)
      .catch(() => setDatasets([]))
      .finally(() => setLoading(false));
  }, [baseUrl, fetchWithHeaders]);

  useEffect(() => {
    refresh();
  }, [refresh]);

  return { datasets, loading, refresh };
};
